"""Teaching control-allocation utilities.

The allocator maps a desired three-axis moment vector to four simple actuator
commands. It uses a damped pseudo-inverse and then applies deflection and rate
limits. This is a small teaching example, not a certified allocation optimizer.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import clamp


Vector3 = tuple[float, float, float]
Vector4 = tuple[float, float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class AllocationParams:
    actuator_names: tuple[str, str, str, str] = ("left_elevon", "right_elevon", "rudder", "body_flap")
    effectiveness: tuple[Vector4, Vector4, Vector4] = (
        (22_000.0, -22_000.0, 2_500.0, 0.0),
        (-8_000.0, -8_000.0, 800.0, -18_000.0),
        (1_600.0, -1_600.0, 21_000.0, 1_200.0),
    )
    efficiency: Vector4 = (1.0, 1.0, 1.0, 1.0)
    deflection_limit: float = math.radians(18.0)
    deflection_rate_limit: float = math.radians(85.0)
    regularization: float = 2.5e5
    reference_deflection: Vector4 = (0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class MomentDemand:
    mx: float
    my: float
    mz: float


@dataclass(frozen=True)
class AllocatorState:
    left_elevon: float = 0.0
    right_elevon: float = 0.0
    rudder: float = 0.0
    body_flap: float = 0.0


@dataclass(frozen=True)
class AllocationResult:
    target: AllocatorState
    actual: AllocatorState
    achieved: MomentDemand
    residual: MomentDemand
    residual_norm: float
    saturated: bool
    rate_limited: bool


def state_to_tuple(state: AllocatorState) -> Vector4:
    return (state.left_elevon, state.right_elevon, state.rudder, state.body_flap)


def tuple_to_state(values: Vector4) -> AllocatorState:
    return AllocatorState(values[0], values[1], values[2], values[3])


def demand_to_tuple(demand: MomentDemand) -> Vector3:
    return (demand.mx, demand.my, demand.mz)


def tuple_to_demand(values: Vector3) -> MomentDemand:
    return MomentDemand(values[0], values[1], values[2])


def effective_matrix(params: AllocationParams) -> tuple[Vector4, Vector4, Vector4]:
    rows: list[Vector4] = []
    for row in params.effectiveness:
        rows.append(
            (
                row[0] * params.efficiency[0],
                row[1] * params.efficiency[1],
                row[2] * params.efficiency[2],
                row[3] * params.efficiency[3],
            )
        )
    return (rows[0], rows[1], rows[2])


def mat_vec_3x4(matrix: tuple[Vector4, Vector4, Vector4], vector: Vector4) -> Vector3:
    return tuple(sum(matrix[row][col] * vector[col] for col in range(4)) for row in range(3))  # type: ignore[return-value]


def _gram_3x3(matrix: tuple[Vector4, Vector4, Vector4], regularization: float) -> Matrix3:
    rows: list[Vector3] = []
    for i in range(3):
        row: list[float] = []
        for j in range(3):
            value = sum(matrix[i][col] * matrix[j][col] for col in range(4))
            if i == j:
                value += regularization
            row.append(value)
        rows.append((row[0], row[1], row[2]))
    return (rows[0], rows[1], rows[2])


def solve_3x3(matrix: Matrix3, rhs: Vector3) -> Vector3:
    a = [[matrix[i][j] for j in range(3)] + [rhs[i]] for i in range(3)]
    for pivot in range(3):
        best = max(range(pivot, 3), key=lambda row: abs(a[row][pivot]))
        if abs(a[best][pivot]) < 1e-12:
            raise ValueError("singular 3x3 system in control allocation")
        if best != pivot:
            a[pivot], a[best] = a[best], a[pivot]
        scale = a[pivot][pivot]
        for col in range(pivot, 4):
            a[pivot][col] /= scale
        for row in range(3):
            if row == pivot:
                continue
            factor = a[row][pivot]
            for col in range(pivot, 4):
                a[row][col] -= factor * a[pivot][col]
    return (a[0][3], a[1][3], a[2][3])


def damped_pseudoinverse_target(demand: MomentDemand, params: AllocationParams) -> AllocatorState:
    matrix = effective_matrix(params)
    reference = params.reference_deflection
    reference_moment = mat_vec_3x4(matrix, reference)
    demand_values = demand_to_tuple(demand)
    rhs: Vector3 = (
        demand_values[0] - reference_moment[0],
        demand_values[1] - reference_moment[1],
        demand_values[2] - reference_moment[2],
    )
    weights = solve_3x3(_gram_3x3(matrix, params.regularization), rhs)
    values = []
    for col in range(4):
        correction = sum(matrix[row][col] * weights[row] for row in range(3))
        values.append(reference[col] + correction)
    return tuple_to_state((values[0], values[1], values[2], values[3]))


def apply_limits(
    previous: AllocatorState,
    target: AllocatorState,
    dt: float,
    params: AllocationParams,
) -> tuple[AllocatorState, bool, bool]:
    previous_values = state_to_tuple(previous)
    target_values = state_to_tuple(target)
    actual: list[float] = []
    saturated = False
    rate_limited = False
    max_step = params.deflection_rate_limit * max(dt, 0.0)
    for previous_value, target_value in zip(previous_values, target_values):
        limited_target = clamp(target_value, -params.deflection_limit, params.deflection_limit)
        saturated = saturated or abs(target_value - limited_target) > 1e-12
        delta = limited_target - previous_value
        limited_delta = clamp(delta, -max_step, max_step)
        rate_limited = rate_limited or abs(delta - limited_delta) > 1e-12
        actual.append(clamp(previous_value + limited_delta, -params.deflection_limit, params.deflection_limit))
    return tuple_to_state((actual[0], actual[1], actual[2], actual[3])), saturated, rate_limited


def allocate_moment(
    demand: MomentDemand,
    previous: AllocatorState,
    dt: float,
    params: AllocationParams | None = None,
) -> AllocationResult:
    params = params or AllocationParams()
    target = damped_pseudoinverse_target(demand, params)
    actual, saturated, rate_limited = apply_limits(previous, target, dt, params)
    achieved_tuple = mat_vec_3x4(effective_matrix(params), state_to_tuple(actual))
    demand_tuple = demand_to_tuple(demand)
    residual_tuple: Vector3 = (
        demand_tuple[0] - achieved_tuple[0],
        demand_tuple[1] - achieved_tuple[1],
        demand_tuple[2] - achieved_tuple[2],
    )
    residual_norm = math.sqrt(sum(value * value for value in residual_tuple))
    return AllocationResult(
        target=target,
        actual=actual,
        achieved=tuple_to_demand(achieved_tuple),
        residual=tuple_to_demand(residual_tuple),
        residual_norm=residual_norm,
        saturated=saturated,
        rate_limited=rate_limited,
    )
