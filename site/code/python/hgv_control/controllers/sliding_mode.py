"""Teaching-oriented sliding-mode controller.

The controller uses a boundary-layer sliding surface for the alpha channel:

    s = (q - gamma_dot) + lambda_alpha * (alpha - alpha_cmd)

The switching term is softened with saturation to keep the example compatible
with actuator rate limits.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import (
    LongitudinalParams,
    State,
    aero_forces,
    clamp,
    derivatives,
    dynamic_pressure,
)


def sat(value: float) -> float:
    return clamp(value, -1.0, 1.0)


@dataclass
class SlidingModeController:
    h_gain: float = 2.8e-4
    gamma_gain: float = 3.0
    lambda_alpha: float = 2.2
    reaching_gain: float = 0.20
    boundary_layer: float = 0.08
    v_gain: float = 0.045
    trim_throttle: float = 0.62
    alpha_trim: float = math.radians(2.0)
    qdot_limit: float = math.radians(55.0)

    def _throttle_command(
        self,
        state: State,
        velocity_ref: float,
        params: LongitudinalParams,
    ) -> float:
        _, drag, _, _ = aero_forces(state, self.trim_throttle, params)
        vdot_cmd = clamp(self.v_gain * (velocity_ref - state.velocity), -7.0, 7.0)
        thrust_required = params.mass * (vdot_cmd + params.gravity * math.sin(state.gamma)) + drag
        denom = params.thrust_max * max(0.2, math.cos(state.alpha))
        return clamp(thrust_required / denom, params.throttle_min, params.throttle_max)

    def _alpha_command(self, state: State, altitude_ref: float) -> float:
        gamma_cmd = clamp(self.h_gain * (altitude_ref - state.altitude), math.radians(-7.0), math.radians(7.0))
        alpha_cmd = self.alpha_trim + clamp(
            self.gamma_gain * (gamma_cmd - state.gamma),
            math.radians(-5.0),
            math.radians(5.0),
        )
        return clamp(alpha_cmd, math.radians(-6.0), math.radians(10.0))

    def _elevator_command(
        self,
        state: State,
        alpha_cmd: float,
        throttle_cmd: float,
        params: LongitudinalParams,
    ) -> float:
        gamma_dot = derivatives(state, state.elevator, throttle_cmd, params).gamma
        alpha_dot = state.pitch_rate - gamma_dot
        surface = alpha_dot + self.lambda_alpha * (state.alpha - alpha_cmd)
        qdot_cmd = -self.lambda_alpha * alpha_dot - self.reaching_gain * sat(surface / self.boundary_layer)
        qdot_cmd = clamp(qdot_cmd, -self.qdot_limit, self.qdot_limit)

        qbar = max(1.0, dynamic_pressure(state, params))
        velocity = max(100.0, state.velocity)
        moment_scale = qbar * params.ref_area * params.ref_chord / params.iy
        a_q = moment_scale * (
            params.cm_alpha * state.alpha
            + params.cm_q * (params.ref_chord / (2.0 * velocity)) * state.pitch_rate
        )
        b_q = moment_scale * params.cm_delta
        if abs(b_q) < 1e-6:
            return clamp(state.elevator, -params.delta_limit, params.delta_limit)
        return clamp((qdot_cmd - a_q) / b_q, -params.delta_limit, params.delta_limit)

    def command(
        self,
        state: State,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
    ) -> tuple[float, float]:
        throttle_cmd = self._throttle_command(state, velocity_ref, params)
        alpha_cmd = self._alpha_command(state, altitude_ref)
        delta_cmd = self._elevator_command(state, alpha_cmd, throttle_cmd, params)
        return delta_cmd, throttle_cmd
