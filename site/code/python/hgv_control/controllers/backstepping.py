"""Teaching-oriented command-filter-free backstepping controller.

This controller follows the recursive structure in the tutorial:

altitude error -> gamma command -> alpha command -> pitch-rate command
-> elevator command.

It remains a compact teaching example, not a full research-grade backstepping
implementation. In particular, reference derivatives and actuator compensation
are simplified so the control chain stays readable.
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


def _state_with_alpha(state: State, alpha: float) -> State:
    return State(
        velocity=state.velocity,
        altitude=state.altitude,
        gamma=state.gamma,
        alpha=alpha,
        pitch_rate=state.pitch_rate,
        theta=state.theta,
        elevator=state.elevator,
        elevator_rate=state.elevator_rate,
    )


@dataclass
class BacksteppingController:
    altitude_gain: float = 0.075
    gamma_gain: float = 1.8
    alpha_gain: float = 1.2
    q_gain: float = 1.5
    velocity_gain: float = 0.05
    trim_throttle: float = 0.62
    alpha_min: float = math.radians(-5.0)
    alpha_max: float = math.radians(8.0)
    q_cmd_limit: float = math.radians(18.0)
    qdot_cmd_limit: float = math.radians(10.0)

    def _reference_rates(self, altitude_ref: float, velocity_ref: float) -> tuple[float, float]:
        altitude_rate_ref = -900.0 / 35.0 if altitude_ref > 29_100.0 else 0.0
        velocity_rate_ref = 80.0 / 25.0 if velocity_ref < 1_830.0 else 0.0
        return altitude_rate_ref, velocity_rate_ref

    def _throttle_command(
        self,
        state: State,
        velocity_ref: float,
        velocity_rate_ref: float,
        params: LongitudinalParams,
    ) -> float:
        _, drag, _, _ = aero_forces(state, self.trim_throttle, params)
        vdot_cmd = velocity_rate_ref - self.velocity_gain * (state.velocity - velocity_ref)
        vdot_cmd = clamp(vdot_cmd, -8.0, 8.0)
        thrust_required = params.mass * (vdot_cmd + params.gravity * math.sin(state.gamma)) + drag
        denom = params.thrust_max * max(0.2, math.cos(state.alpha))
        return clamp(thrust_required / denom, params.throttle_min, params.throttle_max)

    def _gamma_command(
        self,
        state: State,
        altitude_ref: float,
        altitude_rate_ref: float,
    ) -> float:
        velocity = max(100.0, state.velocity)
        h_error = state.altitude - altitude_ref
        sine_gamma = (altitude_rate_ref - self.altitude_gain * h_error) / velocity
        sine_gamma = clamp(sine_gamma, math.sin(math.radians(-7.0)), math.sin(math.radians(7.0)))
        return math.asin(sine_gamma)

    def _alpha_command(
        self,
        state: State,
        gamma_cmd: float,
        throttle_cmd: float,
        params: LongitudinalParams,
    ) -> float:
        gamma_dot_cmd = -self.gamma_gain * (state.gamma - gamma_cmd)
        gamma_dot_cmd = clamp(gamma_dot_cmd, math.radians(-7.0), math.radians(7.0))

        lower = self.alpha_min
        upper = self.alpha_max

        def gamma_dot(alpha: float) -> float:
            trial = _state_with_alpha(state, alpha)
            return derivatives(trial, state.elevator, throttle_cmd, params).gamma

        low_value = gamma_dot(lower)
        high_value = gamma_dot(upper)
        if gamma_dot_cmd <= min(low_value, high_value):
            return lower if low_value < high_value else upper
        if gamma_dot_cmd >= max(low_value, high_value):
            return upper if high_value > low_value else lower

        lo = lower
        hi = upper
        increasing = high_value >= low_value
        for _ in range(28):
            mid = 0.5 * (lo + hi)
            value = gamma_dot(mid)
            if (value < gamma_dot_cmd) == increasing:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def _elevator_command(
        self,
        state: State,
        alpha_cmd: float,
        throttle_cmd: float,
        params: LongitudinalParams,
    ) -> float:
        gamma_dot = derivatives(state, state.elevator, throttle_cmd, params).gamma
        q_cmd = gamma_dot + self.alpha_gain * (alpha_cmd - state.alpha)
        q_cmd = clamp(q_cmd, -self.q_cmd_limit, self.q_cmd_limit)
        qdot_cmd = clamp(self.q_gain * (q_cmd - state.pitch_rate), -self.qdot_cmd_limit, self.qdot_cmd_limit)

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
        altitude_rate_ref, velocity_rate_ref = self._reference_rates(altitude_ref, velocity_ref)
        throttle_cmd = self._throttle_command(state, velocity_ref, velocity_rate_ref, params)
        gamma_cmd = self._gamma_command(state, altitude_ref, altitude_rate_ref)
        alpha_cmd = self._alpha_command(state, gamma_cmd, throttle_cmd, params)
        delta_cmd = self._elevator_command(state, alpha_cmd, throttle_cmd, params)
        return delta_cmd, throttle_cmd
