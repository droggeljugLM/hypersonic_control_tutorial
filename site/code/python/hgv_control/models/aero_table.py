"""Teaching aerodynamic coefficient-table utilities.

The table is deliberately small and synthetic. It teaches interpolation,
alpha/beta bookkeeping, and actuator-effect signs before moving to a real
wind-tunnel or CFD coefficient database.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.attitude import AttitudeState
from hgv_control.models.control_allocation import AllocatorState
from hgv_control.models.longitudinal import clamp
from hgv_control.models.point_mass_3d import PointMass3DState, atmosphere_density_3d, wrap_angle


Table2D = tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class AeroTableParams:
    mach_grid: tuple[float, ...] = (5.0, 7.0, 9.0, 12.0)
    alpha_grid_rad: tuple[float, ...] = tuple(math.radians(value) for value in (-6.0, 0.0, 4.0, 8.0, 12.0, 16.0))
    ref_area: float = 1.2
    ref_length: float = 4.6
    speed_of_sound_m_s: float = 295.0
    max_alpha_rad: float = math.radians(18.0)
    max_beta_rad: float = math.radians(12.0)
    cy_beta: float = -0.18
    cl_beta: float = -0.08
    cn_beta: float = 0.12
    elevon_roll_gain: float = 0.34
    elevon_pitch_gain: float = -0.10
    rudder_yaw_gain: float = 0.26
    rudder_side_gain: float = 0.09
    body_flap_pitch_gain: float = -0.32
    body_flap_drag_gain: float = 0.08


@dataclass(frozen=True)
class AeroAngles:
    alpha: float
    beta: float


@dataclass(frozen=True)
class AeroCoefficients:
    clift: float
    cdrag: float
    cy: float
    cl_roll: float
    cm_pitch: float
    cn_yaw: float


@dataclass(frozen=True)
class AeroLoads:
    lift: float
    drag: float
    side_force: float
    mx: float
    my: float
    mz: float


@dataclass(frozen=True)
class AeroEvaluation:
    coefficients: AeroCoefficients
    loads: AeroLoads
    mach: float
    qbar: float
    alpha: float
    beta: float


def _base_clift(mach: float, alpha: float) -> float:
    return 0.10 + 1.85 * alpha - 0.018 * (mach - 7.0)


def _base_cdrag(mach: float, alpha: float) -> float:
    return 0.060 + 0.38 * alpha * alpha + 0.0025 * (mach - 7.0) * (mach - 7.0)


def _base_cm_pitch(_mach: float, alpha: float) -> float:
    alpha_trim = math.radians(4.0)
    return -0.58 * (alpha - alpha_trim)


def _build_table(params: AeroTableParams, fn) -> Table2D:
    return tuple(tuple(fn(mach, alpha) for alpha in params.alpha_grid_rad) for mach in params.mach_grid)


def _bracket(grid: tuple[float, ...], value: float) -> tuple[int, float]:
    if len(grid) < 2:
        raise ValueError("interpolation grid must contain at least two points")
    if value <= grid[0]:
        return 0, 0.0
    if value >= grid[-1]:
        return len(grid) - 2, 1.0
    for index in range(len(grid) - 1):
        lo = grid[index]
        hi = grid[index + 1]
        if lo <= value <= hi:
            return index, (value - lo) / (hi - lo)
    return len(grid) - 2, 1.0


def interpolate_alpha_mach(table: Table2D, mach: float, alpha: float, params: AeroTableParams | None = None) -> float:
    params = params or AeroTableParams()
    if len(table) != len(params.mach_grid):
        raise ValueError("table row count must match mach grid")
    if any(len(row) != len(params.alpha_grid_rad) for row in table):
        raise ValueError("table column count must match alpha grid")
    mach_index, mach_fraction = _bracket(params.mach_grid, mach)
    alpha_index, alpha_fraction = _bracket(params.alpha_grid_rad, alpha)
    f00 = table[mach_index][alpha_index]
    f10 = table[mach_index + 1][alpha_index]
    f01 = table[mach_index][alpha_index + 1]
    f11 = table[mach_index + 1][alpha_index + 1]
    lower = f00 + mach_fraction * (f10 - f00)
    upper = f01 + mach_fraction * (f11 - f01)
    return lower + alpha_fraction * (upper - lower)


def default_tables(params: AeroTableParams | None = None) -> tuple[Table2D, Table2D, Table2D]:
    params = params or AeroTableParams()
    return (
        _build_table(params, _base_clift),
        _build_table(params, _base_cdrag),
        _build_table(params, _base_cm_pitch),
    )


def mach_from_velocity(velocity_m_s: float, params: AeroTableParams | None = None) -> float:
    params = params or AeroTableParams()
    return max(0.1, velocity_m_s / params.speed_of_sound_m_s)


def qbar_from_state(state: PointMass3DState, point_params, _params: AeroTableParams | None = None) -> float:
    rho = atmosphere_density_3d(state.altitude, point_params)
    return 0.5 * rho * state.velocity * state.velocity


def angles_from_attitude(
    point_state: PointMass3DState,
    attitude_state: AttitudeState,
    params: AeroTableParams | None = None,
) -> AeroAngles:
    params = params or AeroTableParams()
    alpha = clamp(wrap_angle(attitude_state.theta - point_state.gamma), -params.max_alpha_rad, params.max_alpha_rad)
    beta = clamp(
        wrap_angle(attitude_state.psi - point_state.heading) * math.cos(point_state.gamma),
        -params.max_beta_rad,
        params.max_beta_rad,
    )
    return AeroAngles(alpha=alpha, beta=beta)


def evaluate_aero(
    mach: float,
    qbar: float,
    alpha: float,
    beta: float,
    control: AllocatorState | None = None,
    params: AeroTableParams | None = None,
) -> AeroEvaluation:
    params = params or AeroTableParams()
    control = control or AllocatorState()
    alpha = clamp(alpha, -params.max_alpha_rad, params.max_alpha_rad)
    beta = clamp(beta, -params.max_beta_rad, params.max_beta_rad)
    clift_table, cdrag_table, cm_table = default_tables(params)
    clift = interpolate_alpha_mach(clift_table, mach, alpha, params)
    cdrag = interpolate_alpha_mach(cdrag_table, mach, alpha, params)
    cm_pitch = interpolate_alpha_mach(cm_table, mach, alpha, params)

    elevon_sum = control.left_elevon + control.right_elevon
    elevon_diff = control.left_elevon - control.right_elevon
    clift += -0.04 * elevon_sum - 0.05 * control.body_flap
    cdrag += params.body_flap_drag_gain * abs(control.body_flap) + 0.018 * abs(beta)
    cy = params.cy_beta * beta + params.rudder_side_gain * control.rudder
    cl_roll = params.cl_beta * beta + params.elevon_roll_gain * elevon_diff
    cm_pitch += params.elevon_pitch_gain * elevon_sum + params.body_flap_pitch_gain * control.body_flap
    cn_yaw = params.cn_beta * beta + params.rudder_yaw_gain * control.rudder

    force_scale = qbar * params.ref_area
    moment_scale = force_scale * params.ref_length
    coefficients = AeroCoefficients(
        clift=clift,
        cdrag=max(0.0, cdrag),
        cy=cy,
        cl_roll=cl_roll,
        cm_pitch=cm_pitch,
        cn_yaw=cn_yaw,
    )
    loads = AeroLoads(
        lift=force_scale * coefficients.clift,
        drag=force_scale * coefficients.cdrag,
        side_force=force_scale * coefficients.cy,
        mx=moment_scale * coefficients.cl_roll,
        my=moment_scale * coefficients.cm_pitch,
        mz=moment_scale * coefficients.cn_yaw,
    )
    return AeroEvaluation(coefficients=coefficients, loads=loads, mach=mach, qbar=qbar, alpha=alpha, beta=beta)
