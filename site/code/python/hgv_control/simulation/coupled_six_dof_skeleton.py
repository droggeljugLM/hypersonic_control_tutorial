"""Run a teaching coupled six-DOF-style skeleton.

This example closes the loop between guidance, attitude, control allocation,
teaching aerodynamic tables, and 3-D point-mass translation. Allocated actuator
deflections generate teaching aerodynamic forces and moments. Those moments
drive the attitude model, and the aerodynamic forces plus achieved attitude
drive the point-mass flight-path and heading rates.

It is still a teaching skeleton: its aerodynamic tables are synthetic, it uses
small-angle teaching alpha/beta bookkeeping, and its quaternion/DCM fields are
kinematic health checks rather than a full six-DOF rigid-body plant.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from hgv_control.metrics.summary import rms, violation_integral
from hgv_control.models.aero_table import (
    AeroEvaluation,
    AeroTableParams,
    angles_from_attitude,
    evaluate_aero,
    mach_from_velocity,
)
from hgv_control.models.attitude import AttitudeParams, AttitudeState, MomentCommand, rk4_step as attitude_step
from hgv_control.models.control_allocation import (
    AllocationParams,
    AllocatorState,
    MomentDemand,
    allocate_moment,
    state_to_tuple,
)
from hgv_control.models.frames import dot, flight_path_basis, force_path_rates, wind_forces_to_enu
from hgv_control.models.longitudinal import clamp
from hgv_control.models.point_mass_3d import (
    PointMass3DCommand,
    PointMass3DParams,
    PointMass3DState,
    dynamic_pressure_3d,
    load_factor_3d,
    rk4_step as point_mass_step,
    wrap_angle,
)
from hgv_control.models.rigid_body_kinematics import (
    Quaternion,
    dcm_matvec,
    dcm_orthogonality_error,
    euler_to_quaternion,
    integrate_quaternion_euler,
    quaternion_norm,
    quaternion_to_dcm_body_to_enu,
    wind_forces_to_enu_dcm,
)
from hgv_control.simulation.attitude_inner_loop import pd_moment_command
from hgv_control.simulation.guidance_3d import GuidanceTarget3D, guidance_command, make_initial_state
from hgv_control.simulation.guidance_attitude_interface import (
    InterfaceParams,
    guidance_to_attitude_reference,
    make_attitude_state,
)


def achieved_point_command(
    point_state: PointMass3DState,
    attitude_state: AttitudeState,
    guidance_throttle: float,
    point_params: PointMass3DParams,
    interface_params: InterfaceParams,
) -> PointMass3DCommand:
    gamma_from_attitude = attitude_state.theta - interface_params.alpha_trim
    gamma_error = gamma_from_attitude - point_state.gamma
    gamma_rate = clamp(0.32 * gamma_error + 0.08 * attitude_state.q, -point_params.gamma_rate_limit, point_params.gamma_rate_limit)

    heading_error = wrap_angle(attitude_state.psi - point_state.heading)
    bank_heading_rate = point_params.gravity * math.tan(attitude_state.phi) / max(point_state.velocity * math.cos(point_state.gamma), 100.0)
    heading_rate = clamp(
        bank_heading_rate + 0.08 * heading_error + 0.05 * attitude_state.r,
        -point_params.heading_rate_limit,
        point_params.heading_rate_limit,
    )
    return PointMass3DCommand(gamma_rate=gamma_rate, heading_rate=heading_rate, throttle=guidance_throttle)


def aero_coupled_point_command(
    point_state: PointMass3DState,
    attitude_state: AttitudeState,
    guidance_throttle: float,
    point_params: PointMass3DParams,
    interface_params: InterfaceParams,
    aero: AeroEvaluation,
) -> PointMass3DCommand:
    gamma_from_attitude = attitude_state.theta - interface_params.alpha_trim
    gamma_error = gamma_from_attitude - point_state.gamma
    force_enu = wind_forces_to_enu(
        aero.loads.lift,
        aero.loads.drag,
        aero.loads.side_force,
        point_state.gamma,
        point_state.heading,
        attitude_state.phi,
    )
    _speed_dot, force_gamma_rate, force_heading_rate = force_path_rates(
        force_enu,
        point_params.mass,
        point_state.velocity,
        point_state.gamma,
        point_state.heading,
        point_params.gravity,
    )
    heading_error = wrap_angle(attitude_state.psi - point_state.heading)
    gamma_rate = clamp(
        force_gamma_rate + 0.18 * gamma_error + 0.05 * attitude_state.q,
        -point_params.gamma_rate_limit,
        point_params.gamma_rate_limit,
    )
    heading_rate = clamp(
        force_heading_rate + 0.04 * heading_error + 0.03 * attitude_state.r,
        -point_params.heading_rate_limit,
        point_params.heading_rate_limit,
    )
    return PointMass3DCommand(gamma_rate=gamma_rate, heading_rate=heading_rate, throttle=guidance_throttle)


def aero_force_projections(
    point_state: PointMass3DState,
    attitude_state: AttitudeState,
    point_params: PointMass3DParams,
    aero: AeroEvaluation,
) -> dict[str, float]:
    force_enu = wind_forces_to_enu(
        aero.loads.lift,
        aero.loads.drag,
        aero.loads.side_force,
        point_state.gamma,
        point_state.heading,
        attitude_state.phi,
    )
    velocity_axis, normal_axis, heading_axis = flight_path_basis(point_state.gamma, point_state.heading)
    speed_dot, gamma_rate_force, heading_rate_force = force_path_rates(
        force_enu,
        point_params.mass,
        point_state.velocity,
        point_state.gamma,
        point_state.heading,
        point_params.gravity,
    )
    return {
        "force_east": force_enu[0],
        "force_north": force_enu[1],
        "force_up": force_enu[2],
        "force_tangent": dot(force_enu, velocity_axis),
        "force_normal": dot(force_enu, normal_axis),
        "force_lateral": dot(force_enu, heading_axis),
        "speed_dot_force_m_s2": speed_dot,
        "gamma_rate_force_rad_s": gamma_rate_force,
        "heading_rate_force_rad_s": heading_rate_force,
    }


def attitude_kinematics_fields(quaternion: Quaternion) -> dict[str, float]:
    dcm_body_to_enu = quaternion_to_dcm_body_to_enu(quaternion)
    body_x_enu = dcm_matvec(dcm_body_to_enu, (1.0, 0.0, 0.0))
    return {
        "quat_w": quaternion[0],
        "quat_x": quaternion[1],
        "quat_y": quaternion[2],
        "quat_z": quaternion[3],
        "quaternion_norm_error": abs(quaternion_norm(quaternion) - 1.0),
        "dcm_orthogonality_error": dcm_orthogonality_error(dcm_body_to_enu),
        "body_x_east": body_x_enu[0],
        "body_x_north": body_x_enu[1],
        "body_x_up": body_x_enu[2],
    }


def dcm_force_transform_fields(
    quaternion: Quaternion,
    aero: AeroEvaluation,
    force_projection: dict[str, float],
) -> dict[str, float]:
    dcm_body_to_enu = quaternion_to_dcm_body_to_enu(quaternion)
    body_force, dcm_force_enu = wind_forces_to_enu_dcm(
        aero.loads.lift,
        aero.loads.drag,
        aero.loads.side_force,
        aero.alpha,
        aero.beta,
        dcm_body_to_enu,
    )
    delta_east = dcm_force_enu[0] - force_projection["force_east"]
    delta_north = dcm_force_enu[1] - force_projection["force_north"]
    delta_up = dcm_force_enu[2] - force_projection["force_up"]
    return {
        "body_force_x": body_force[0],
        "body_force_y": body_force[1],
        "body_force_z": body_force[2],
        "dcm_force_east": dcm_force_enu[0],
        "dcm_force_north": dcm_force_enu[1],
        "dcm_force_up": dcm_force_enu[2],
        "dcm_force_delta_east": delta_east,
        "dcm_force_delta_north": delta_north,
        "dcm_force_delta_up": delta_up,
        "dcm_force_delta_norm": math.sqrt(delta_east * delta_east + delta_north * delta_north + delta_up * delta_up),
    }


def summarize_coupled(
    trace: list[dict[str, float]],
    target: GuidanceTarget3D,
    point_params: PointMass3DParams,
    allocation_params: AllocationParams,
    dt: float,
) -> dict[str, float | bool]:
    last = trace[-1]
    horizontal_miss = math.hypot(last["east"] - target.east, last["north"] - target.north)
    altitude_error = last["altitude"] - target.altitude
    terminal_range = math.sqrt(horizontal_miss * horizontal_miss + altitude_error * altitude_error)
    speed_error = last["velocity"] - target.velocity
    q_values = [row["qbar"] for row in trace]
    load_values = [row["load_factor"] for row in trace]
    attitude_errors = [
        math.sqrt((row["roll_error"] ** 2 + row["pitch_error"] ** 2 + row["yaw_error"] ** 2) / 3.0)
        for row in trace
    ]
    gamma_rate_errors = [row["gamma_rate_achieved"] - row["gamma_rate_cmd"] for row in trace]
    heading_rate_errors = [row["heading_rate_achieved"] - row["heading_rate_cmd"] for row in trace]
    residual_values = [row["allocation_residual_norm"] for row in trace]
    aero_moment_errors = [
        math.sqrt(
            (
                (row["mx_aero"] - row["mx_cmd"]) ** 2
                + (row["my_aero"] - row["my_cmd"]) ** 2
                + (row["mz_aero"] - row["mz_cmd"]) ** 2
            )
            / 3.0
        )
        for row in trace
    ]
    alpha_values = [abs(row["alpha_rad"]) for row in trace]
    beta_values = [abs(row["beta_rad"]) for row in trace]
    force_projection_values = [math.sqrt(row["force_normal"] ** 2 + row["force_lateral"] ** 2) for row in trace]
    force_transform_delta_values = [row["dcm_force_delta_norm"] for row in trace]
    quaternion_norm_errors = [row["quaternion_norm_error"] for row in trace]
    dcm_orthogonality_errors = [row["dcm_orthogonality_error"] for row in trace]
    deflection_values = [
        abs(value)
        for row in trace
        for value in (row["left_elevon"], row["right_elevon"], row["rudder"], row["body_flap"])
    ]
    q_violation = violation_integral(q_values, point_params.q_limit, dt)
    load_violation = violation_integral(load_values, point_params.load_factor_limit, dt)
    attitude_rms = rms(attitude_errors)
    allocation_residual_rms = rms(residual_values)
    deflection_max = max(deflection_values) if deflection_values else 0.0
    terminal_pass = terminal_range <= 15_000.0 and abs(speed_error) <= 240.0
    path_pass = q_violation <= 1.0 and load_violation <= 0.1
    attitude_pass = attitude_rms <= math.radians(10.0)
    allocation_pass = allocation_residual_rms <= 1_800.0 and deflection_max <= allocation_params.deflection_limit + 1e-9
    aero_feedback_pass = rms(aero_moment_errors) <= 5_500.0 and max(alpha_values) <= math.radians(18.0) + 1e-9 and max(beta_values) <= math.radians(12.0) + 1e-9
    kinematics_pass = max(quaternion_norm_errors) <= 1e-9 and max(dcm_orthogonality_errors) <= 1e-9
    force_transform_pass = rms(force_transform_delta_values) <= 15_000.0
    return {
        "terminal_range_m": terminal_range,
        "horizontal_miss_m": horizontal_miss,
        "terminal_altitude_error_m": altitude_error,
        "terminal_speed_error_m_s": speed_error,
        "q_violation_integral": q_violation,
        "load_factor_violation_integral": load_violation,
        "attitude_rms_rad": attitude_rms,
        "gamma_rate_error_rms_rad_s": rms(gamma_rate_errors),
        "heading_rate_error_rms_rad_s": rms(heading_rate_errors),
        "allocation_residual_rms_nm": allocation_residual_rms,
        "allocation_residual_max_nm": max(residual_values) if residual_values else 0.0,
        "aero_moment_error_rms_nm": rms(aero_moment_errors),
        "aero_force_projection_rms_n": rms(force_projection_values),
        "dcm_force_delta_rms_n": rms(force_transform_delta_values),
        "dcm_force_delta_max_n": max(force_transform_delta_values) if force_transform_delta_values else 0.0,
        "quaternion_norm_error_max": max(quaternion_norm_errors) if quaternion_norm_errors else 0.0,
        "dcm_orthogonality_error_max": max(dcm_orthogonality_errors) if dcm_orthogonality_errors else 0.0,
        "alpha_max_rad": max(alpha_values) if alpha_values else 0.0,
        "beta_max_rad": max(beta_values) if beta_values else 0.0,
        "deflection_max_rad": deflection_max,
        "terminal_pass": terminal_pass,
        "path_pass": path_pass,
        "attitude_pass": attitude_pass,
        "allocation_pass": allocation_pass,
        "aero_feedback_pass": aero_feedback_pass,
        "kinematics_pass": kinematics_pass,
        "force_transform_pass": force_transform_pass,
        "pass": terminal_pass
        and path_pass
        and attitude_pass
        and allocation_pass
        and aero_feedback_pass
        and kinematics_pass
        and force_transform_pass,
    }


def run(
    duration: float = 34.0,
    dt: float = 0.05,
    point_params: PointMass3DParams | None = None,
    attitude_params: AttitudeParams | None = None,
    allocation_params: AllocationParams | None = None,
    aero_params: AeroTableParams | None = None,
    interface_params: InterfaceParams | None = None,
    target: GuidanceTarget3D | None = None,
    initial_point_state: PointMass3DState | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    point_params = point_params or PointMass3DParams()
    attitude_params = attitude_params or AttitudeParams()
    allocation_params = allocation_params or AllocationParams()
    aero_params = aero_params or AeroTableParams()
    interface_params = interface_params or InterfaceParams()
    target = target or GuidanceTarget3D()
    point_state = initial_point_state or make_initial_state()
    attitude_state = make_attitude_state(point_state, interface_params)
    attitude_quaternion = euler_to_quaternion(attitude_state.phi, attitude_state.theta, attitude_state.psi)
    allocator_state = AllocatorState()
    previous_deflection = state_to_tuple(allocator_state)
    trace: list[dict[str, float]] = []
    steps = int(duration / dt)

    for step in range(steps + 1):
        time_s = step * dt
        guidance = guidance_command(point_state, target, point_params)
        interface = guidance_to_attitude_reference(point_state, target, point_params, interface_params)
        moment_cmd = pd_moment_command(attitude_state, (interface.roll_ref, interface.pitch_ref, interface.yaw_ref), attitude_params)
        allocation = allocate_moment(MomentDemand(moment_cmd.mx, moment_cmd.my, moment_cmd.mz), allocator_state, dt, allocation_params)
        actual_deflection = state_to_tuple(allocation.actual)
        deflection_rates = tuple((actual_deflection[i] - previous_deflection[i]) / dt if step > 0 else 0.0 for i in range(4))
        qbar = dynamic_pressure_3d(point_state, point_params)
        aero_angles = angles_from_attitude(point_state, attitude_state, aero_params)
        aero = evaluate_aero(
            mach_from_velocity(point_state.velocity, aero_params),
            qbar,
            aero_angles.alpha,
            aero_angles.beta,
            allocation.actual,
            aero_params,
        )
        achieved_command = aero_coupled_point_command(
            point_state,
            attitude_state,
            guidance.throttle,
            point_params,
            interface_params,
            aero,
        )
        force_projection = aero_force_projections(point_state, attitude_state, point_params, aero)
        kinematics_fields = attitude_kinematics_fields(attitude_quaternion)
        force_transform_fields = dcm_force_transform_fields(attitude_quaternion, aero, force_projection)
        load_factor = load_factor_3d(point_state, achieved_command, point_params)
        trace.append(
            {
                "time": time_s,
                "east": point_state.east,
                "north": point_state.north,
                "altitude": point_state.altitude,
                "velocity": point_state.velocity,
                "gamma": point_state.gamma,
                "heading": point_state.heading,
                "roll": attitude_state.phi,
                "pitch": attitude_state.theta,
                "yaw": attitude_state.psi,
                "roll_ref": interface.roll_ref,
                "pitch_ref": interface.pitch_ref,
                "yaw_ref": interface.yaw_ref,
                "roll_error": wrap_angle(attitude_state.phi - interface.roll_ref),
                "pitch_error": wrap_angle(attitude_state.theta - interface.pitch_ref),
                "yaw_error": wrap_angle(attitude_state.psi - interface.yaw_ref),
                "gamma_rate_cmd": guidance.gamma_rate,
                "heading_rate_cmd": guidance.heading_rate,
                "gamma_rate_achieved": achieved_command.gamma_rate,
                "heading_rate_achieved": achieved_command.heading_rate,
                "alpha_rad": aero.alpha,
                "beta_rad": aero.beta,
                "clift": aero.coefficients.clift,
                "cdrag": aero.coefficients.cdrag,
                "cy": aero.coefficients.cy,
                "cl_roll": aero.coefficients.cl_roll,
                "cm_pitch": aero.coefficients.cm_pitch,
                "cn_yaw": aero.coefficients.cn_yaw,
                "lift": aero.loads.lift,
                "drag": aero.loads.drag,
                "side_force": aero.loads.side_force,
                **force_projection,
                **kinematics_fields,
                **force_transform_fields,
                "mx_cmd": moment_cmd.mx,
                "my_cmd": moment_cmd.my,
                "mz_cmd": moment_cmd.mz,
                "mx_allocated": allocation.achieved.mx,
                "my_allocated": allocation.achieved.my,
                "mz_allocated": allocation.achieved.mz,
                "mx_aero": aero.loads.mx,
                "my_aero": aero.loads.my,
                "mz_aero": aero.loads.mz,
                "allocation_residual_norm": allocation.residual_norm,
                "left_elevon": allocation.actual.left_elevon,
                "right_elevon": allocation.actual.right_elevon,
                "rudder": allocation.actual.rudder,
                "body_flap": allocation.actual.body_flap,
                "left_elevon_rate": deflection_rates[0],
                "right_elevon_rate": deflection_rates[1],
                "rudder_rate": deflection_rates[2],
                "body_flap_rate": deflection_rates[3],
                "qbar": qbar,
                "load_factor": load_factor,
            }
        )
        previous_deflection = actual_deflection
        allocator_state = allocation.actual
        attitude_quaternion = integrate_quaternion_euler(
            attitude_quaternion,
            (attitude_state.p, attitude_state.q, attitude_state.r),
            dt,
        )
        attitude_state = attitude_step(
            attitude_state,
            MomentCommand(aero.loads.mx, aero.loads.my, aero.loads.mz),
            dt,
            attitude_params,
        )
        point_state = point_mass_step(point_state, achieved_command, dt, point_params)

    return trace, summarize_coupled(trace, target, point_params, allocation_params, dt)


def write_csv(trace: list[dict[str, float]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=34.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    trace, metrics = run(duration=args.duration, dt=args.dt)
    if args.output is not None:
        write_csv(trace, args.output)
    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
