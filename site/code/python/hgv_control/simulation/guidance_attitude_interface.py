"""Run a teaching guidance-to-attitude interface scenario.

This module connects the existing 3-D point-mass guidance command to a
notional attitude reference and control-allocation contract. The point-mass
guidance still drives the translational state directly. The attitude and
allocation layers are checked for command compatibility, not used as a full
six-DOF closed loop.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.metrics.summary import rms, violation_integral
from hgv_control.models.attitude import AttitudeParams, AttitudeState, MomentCommand, rk4_step as attitude_step
from hgv_control.models.control_allocation import (
    AllocationParams,
    AllocatorState,
    MomentDemand,
    allocate_moment,
    state_to_tuple,
)
from hgv_control.models.longitudinal import clamp
from hgv_control.models.point_mass_3d import (
    PointMass3DParams,
    PointMass3DState,
    dynamic_pressure_3d,
    load_factor_3d,
    rk4_step as point_mass_step,
    wrap_angle,
)
from hgv_control.simulation.attitude_inner_loop import pd_moment_command
from hgv_control.simulation.guidance_3d import (
    GuidanceTarget3D,
    guidance_command,
    make_initial_state,
    range_to_target,
)


@dataclass(frozen=True)
class InterfaceParams:
    alpha_trim: float = math.radians(2.0)
    gamma_lead_time: float = 1.4
    heading_lead_time: float = 1.2
    bank_limit: float = math.radians(15.0)
    pitch_limit: float = math.radians(18.0)
    yaw_lead_limit: float = math.radians(10.0)


@dataclass(frozen=True)
class InterfaceCommand:
    roll_ref: float
    pitch_ref: float
    yaw_ref: float
    desired_gamma: float
    desired_heading: float
    gamma_error: float
    heading_error: float
    lateral_accel_cmd: float


def make_attitude_state(point_state: PointMass3DState, interface_params: InterfaceParams | None = None) -> AttitudeState:
    interface_params = interface_params or InterfaceParams()
    return AttitudeState(
        phi=0.0,
        theta=clamp(point_state.gamma + interface_params.alpha_trim, -interface_params.pitch_limit, interface_params.pitch_limit),
        psi=point_state.heading,
        p=0.0,
        q=0.0,
        r=0.0,
        mx=0.0,
        my=0.0,
        mz=0.0,
    )


def guidance_to_attitude_reference(
    point_state: PointMass3DState,
    target: GuidanceTarget3D,
    point_params: PointMass3DParams,
    interface_params: InterfaceParams | None = None,
) -> InterfaceCommand:
    interface_params = interface_params or InterfaceParams()
    east_error = target.east - point_state.east
    north_error = target.north - point_state.north
    horizontal_range, _total_range, altitude_error = range_to_target(point_state, target)
    desired_heading = math.atan2(north_error, east_error)
    desired_gamma = math.atan2(altitude_error, max(horizontal_range, 1.0))
    desired_gamma = clamp(desired_gamma, math.radians(-8.0), math.radians(5.0))
    command = guidance_command(point_state, target, point_params)
    heading_error = wrap_angle(desired_heading - point_state.heading)
    gamma_error = desired_gamma - point_state.gamma
    lateral_accel_cmd = point_state.velocity * math.cos(point_state.gamma) * command.heading_rate
    roll_ref = clamp(math.atan2(lateral_accel_cmd, point_params.gravity), -interface_params.bank_limit, interface_params.bank_limit)
    pitch_ref = clamp(
        point_state.gamma + interface_params.alpha_trim + interface_params.gamma_lead_time * command.gamma_rate,
        -interface_params.pitch_limit,
        interface_params.pitch_limit,
    )
    yaw_lead = clamp(interface_params.heading_lead_time * command.heading_rate, -interface_params.yaw_lead_limit, interface_params.yaw_lead_limit)
    yaw_ref = wrap_angle(point_state.heading + yaw_lead)
    return InterfaceCommand(
        roll_ref=roll_ref,
        pitch_ref=pitch_ref,
        yaw_ref=yaw_ref,
        desired_gamma=desired_gamma,
        desired_heading=desired_heading,
        gamma_error=gamma_error,
        heading_error=heading_error,
        lateral_accel_cmd=lateral_accel_cmd,
    )


def _angle_error(actual: float, reference: float) -> float:
    return wrap_angle(actual - reference)


def summarize_interface(
    trace: list[dict[str, float]],
    point_params: PointMass3DParams,
    allocation_params: AllocationParams,
    dt: float,
) -> dict[str, float | bool]:
    q_values = [row["qbar"] for row in trace]
    load_values = [row["load_factor"] for row in trace]
    gamma_errors = [row["gamma_error"] for row in trace]
    heading_errors = [row["heading_error"] for row in trace]
    attitude_errors = [
        math.sqrt(
            (
                row["roll_error"] * row["roll_error"]
                + row["pitch_error"] * row["pitch_error"]
                + row["yaw_error"] * row["yaw_error"]
            )
            / 3.0
        )
        for row in trace
    ]
    residuals = [row["allocation_residual_norm"] for row in trace]
    deflections = [
        abs(value)
        for row in trace
        for value in (row["left_elevon"], row["right_elevon"], row["rudder"], row["body_flap"])
    ]
    roll_refs = [abs(row["roll_ref"]) for row in trace]
    pitch_refs = [abs(row["pitch_ref"]) for row in trace]
    yaw_refs = [abs(row["yaw_ref"]) for row in trace]
    q_violation = violation_integral(q_values, point_params.q_limit, dt)
    load_violation = violation_integral(load_values, point_params.load_factor_limit, dt)
    attitude_interface_rms = rms(attitude_errors)
    allocation_residual_rms = rms(residuals)
    deflection_max = max(deflections) if deflections else 0.0
    command_pass = max(roll_refs + pitch_refs + yaw_refs) <= math.radians(35.0)
    attitude_pass = attitude_interface_rms <= math.radians(8.0)
    allocation_pass = allocation_residual_rms <= 1_600.0 and deflection_max <= allocation_params.deflection_limit + 1e-9
    path_pass = q_violation <= 1.0 and load_violation <= 0.1
    return {
        "guidance_gamma_error_rms_rad": rms(gamma_errors),
        "guidance_heading_error_rms_rad": rms(heading_errors),
        "attitude_interface_rms_rad": attitude_interface_rms,
        "allocation_residual_rms_nm": allocation_residual_rms,
        "allocation_residual_max_nm": max(residuals) if residuals else 0.0,
        "deflection_max_rad": deflection_max,
        "roll_ref_max_rad": max(roll_refs) if roll_refs else 0.0,
        "pitch_ref_max_rad": max(pitch_refs) if pitch_refs else 0.0,
        "q_violation_integral": q_violation,
        "load_factor_violation_integral": load_violation,
        "command_pass": command_pass,
        "attitude_pass": attitude_pass,
        "allocation_pass": allocation_pass,
        "path_pass": path_pass,
        "pass": command_pass and attitude_pass and allocation_pass and path_pass,
    }


def run(
    duration: float = 34.0,
    dt: float = 0.05,
    point_params: PointMass3DParams | None = None,
    attitude_params: AttitudeParams | None = None,
    allocation_params: AllocationParams | None = None,
    interface_params: InterfaceParams | None = None,
    target: GuidanceTarget3D | None = None,
    initial_point_state: PointMass3DState | None = None,
) -> tuple[list[dict[str, float]], dict[str, float | bool]]:
    point_params = point_params or PointMass3DParams()
    attitude_params = attitude_params or AttitudeParams()
    allocation_params = allocation_params or AllocationParams()
    interface_params = interface_params or InterfaceParams()
    target = target or GuidanceTarget3D()
    point_state = initial_point_state or make_initial_state()
    attitude_state = make_attitude_state(point_state, interface_params)
    allocator_state = AllocatorState()
    previous_deflection = state_to_tuple(allocator_state)
    trace: list[dict[str, float]] = []
    steps = int(duration / dt)

    for step in range(steps + 1):
        time_s = step * dt
        guidance = guidance_command(point_state, target, point_params)
        interface = guidance_to_attitude_reference(point_state, target, point_params, interface_params)
        moment_cmd = pd_moment_command(
            attitude_state,
            (interface.roll_ref, interface.pitch_ref, interface.yaw_ref),
            attitude_params,
        )
        allocation = allocate_moment(
            MomentDemand(moment_cmd.mx, moment_cmd.my, moment_cmd.mz),
            allocator_state,
            dt,
            allocation_params,
        )
        actual_deflection = state_to_tuple(allocation.actual)
        deflection_rates = tuple((actual_deflection[i] - previous_deflection[i]) / dt if step > 0 else 0.0 for i in range(4))
        qbar = dynamic_pressure_3d(point_state, point_params)
        load_factor = load_factor_3d(point_state, guidance, point_params)
        trace.append(
            {
                "time": time_s,
                "east": point_state.east,
                "north": point_state.north,
                "altitude": point_state.altitude,
                "velocity": point_state.velocity,
                "gamma": point_state.gamma,
                "heading": point_state.heading,
                "desired_gamma": interface.desired_gamma,
                "desired_heading": interface.desired_heading,
                "gamma_error": interface.gamma_error,
                "heading_error": interface.heading_error,
                "roll_ref": interface.roll_ref,
                "pitch_ref": interface.pitch_ref,
                "yaw_ref": interface.yaw_ref,
                "roll": attitude_state.phi,
                "pitch": attitude_state.theta,
                "yaw": attitude_state.psi,
                "roll_error": _angle_error(attitude_state.phi, interface.roll_ref),
                "pitch_error": _angle_error(attitude_state.theta, interface.pitch_ref),
                "yaw_error": _angle_error(attitude_state.psi, interface.yaw_ref),
                "gamma_rate_cmd": guidance.gamma_rate,
                "heading_rate_cmd": guidance.heading_rate,
                "lateral_accel_cmd": interface.lateral_accel_cmd,
                "mx_cmd": moment_cmd.mx,
                "my_cmd": moment_cmd.my,
                "mz_cmd": moment_cmd.mz,
                "mx_allocated": allocation.achieved.mx,
                "my_allocated": allocation.achieved.my,
                "mz_allocated": allocation.achieved.mz,
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
        attitude_state = attitude_step(
            attitude_state,
            MomentCommand(allocation.achieved.mx, allocation.achieved.my, allocation.achieved.mz),
            dt,
            attitude_params,
        )
        point_state = point_mass_step(point_state, guidance, dt, point_params)

    return trace, summarize_interface(trace, point_params, allocation_params, dt)


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
