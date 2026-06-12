"""Teaching-oriented discrete LQR controller.

This module intentionally uses only the Python standard library so the tutorial
example remains runnable in a minimal environment.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import (
    LongitudinalParams,
    State,
    clamp,
    derivatives,
)


Matrix = list[list[float]]
Vector = list[float]


def eye(n: int) -> Matrix:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def transpose(a: Matrix) -> Matrix:
    return [list(row) for row in zip(*a)]


def matmul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a_i[k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for a_i in a]


def matvec(a: Matrix, x: Vector) -> Vector:
    return [sum(row[j] * x[j] for j in range(len(x))) for row in a]


def matadd(a: Matrix, b: Matrix) -> Matrix:
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def matsub(a: Matrix, b: Matrix) -> Matrix:
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def diag(values: Vector) -> Matrix:
    return [[values[i] if i == j else 0.0 for j in range(len(values))] for i in range(len(values))]


def inverse(a: Matrix) -> Matrix:
    n = len(a)
    aug = [row[:] + ident[:] for row, ident in zip(a, eye(n))]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise ValueError("singular matrix in LQR solve")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        scale = aug[col][col]
        aug[col] = [value / scale for value in aug[col]]
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            aug[row] = [aug[row][j] - factor * aug[col][j] for j in range(2 * n)]
    return [row[n:] for row in aug]


def state_to_vector(state: State) -> Vector:
    return [
        state.velocity,
        state.altitude,
        state.gamma,
        state.alpha,
        state.pitch_rate,
        state.theta,
        state.elevator,
        state.elevator_rate,
    ]


def vector_to_state(values: Vector) -> State:
    return State(
        velocity=values[0],
        altitude=values[1],
        gamma=values[2],
        alpha=values[3],
        pitch_rate=values[4],
        theta=values[5],
        elevator=values[6],
        elevator_rate=values[7],
    )


def dynamics_vector(values: Vector, controls: Vector, params: LongitudinalParams) -> Vector:
    return state_to_vector(derivatives(vector_to_state(values), controls[0], controls[1], params))


def linearize_continuous(
    state: State,
    controls: Vector,
    params: LongitudinalParams,
    state_steps: Vector,
    control_steps: Vector,
) -> tuple[Matrix, Matrix]:
    x0 = state_to_vector(state)
    n = len(x0)
    m = len(controls)
    a = [[0.0 for _ in range(n)] for _ in range(n)]
    b = [[0.0 for _ in range(m)] for _ in range(n)]

    for col in range(n):
        step = state_steps[col]
        xp = x0[:]
        xm = x0[:]
        xp[col] += step
        xm[col] -= step
        fp = dynamics_vector(xp, controls, params)
        fm = dynamics_vector(xm, controls, params)
        for row in range(n):
            a[row][col] = (fp[row] - fm[row]) / (2.0 * step)

    for col in range(m):
        step = control_steps[col]
        up = controls[:]
        um = controls[:]
        up[col] += step
        um[col] -= step
        fp = dynamics_vector(x0, up, params)
        fm = dynamics_vector(x0, um, params)
        for row in range(n):
            b[row][col] = (fp[row] - fm[row]) / (2.0 * step)
    return a, b


def discretize_euler(a: Matrix, b: Matrix, dt: float) -> tuple[Matrix, Matrix]:
    n = len(a)
    ad = [[(1.0 if i == j else 0.0) + dt * a[i][j] for j in range(n)] for i in range(n)]
    bd = [[dt * value for value in row] for row in b]
    return ad, bd


def dare_gain(ad: Matrix, bd: Matrix, q: Matrix, r: Matrix, iterations: int = 350) -> Matrix:
    p = q
    at = transpose(ad)
    bt = transpose(bd)
    for _ in range(iterations):
        bt_p = matmul(bt, p)
        s = matadd(r, matmul(bt_p, bd))
        s_inv = inverse(s)
        feedback_term = matmul(matmul(matmul(at, p), bd), matmul(s_inv, matmul(bt_p, ad)))
        p_next = matadd(q, matsub(matmul(matmul(at, p), ad), feedback_term))
        p = p_next
    return matmul(matmul(inverse(matadd(r, matmul(matmul(bt, p), bd))), matmul(bt, p)), ad)


@dataclass
class LqrController:
    trim_throttle: float = 0.62
    alpha_trim: float = math.radians(2.0)
    gamma_trim: float = math.radians(-0.5)

    def __post_init__(self) -> None:
        self._gain_cache: dict[tuple[float, float, float, float, float], Matrix] = {}

    def _gain(self, params: LongitudinalParams) -> Matrix:
        key = (
            params.rho0,
            params.thrust_max,
            params.cl_alpha,
            params.cd_alpha2,
            params.actuator_wn,
        )
        if key in self._gain_cache:
            return self._gain_cache[key]

        trim_state = State(
            velocity=1_750.0,
            altitude=30_000.0,
            gamma=self.gamma_trim,
            alpha=self.alpha_trim,
            pitch_rate=0.0,
            theta=self.alpha_trim + self.gamma_trim,
            elevator=0.0,
            elevator_rate=0.0,
        )
        a, b = linearize_continuous(
            trim_state,
            [0.0, self.trim_throttle],
            params,
            state_steps=[1.0, 10.0, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4],
            control_steps=[1e-4, 1e-4],
        )
        ad, bd = discretize_euler(a, b, dt=0.05)
        q = diag([2.0e-5, 1.0e-7, 2.0, 18.0, 2.0, 5.0, 0.4, 0.02])
        r = diag([18.0, 2.0])
        gain = dare_gain(ad, bd, q, r)
        self._gain_cache[key] = gain
        return gain

    def command(
        self,
        state: State,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
    ) -> tuple[float, float]:
        gamma_ref = clamp(1.5e-4 * (altitude_ref - state.altitude), math.radians(-7.0), math.radians(7.0))
        alpha_ref = self.alpha_trim + clamp(0.75 * (gamma_ref - state.gamma), math.radians(-4.0), math.radians(4.0))
        theta_ref = alpha_ref + gamma_ref
        x_ref = [
            velocity_ref,
            altitude_ref,
            gamma_ref,
            alpha_ref,
            0.0,
            theta_ref,
            0.0,
            0.0,
        ]
        error = [state_value - ref_value for state_value, ref_value in zip(state_to_vector(state), x_ref)]
        feedback = matvec(self._gain(params), error)
        delta_cmd = -feedback[0]
        throttle_cmd = self.trim_throttle - feedback[1]

        delta_cmd = clamp(delta_cmd, -params.delta_limit, params.delta_limit)
        throttle_cmd = clamp(throttle_cmd, params.throttle_min, params.throttle_max)
        return delta_cmd, throttle_cmd

