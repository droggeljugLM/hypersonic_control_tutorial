"""Simple cascaded PID-style baseline controller."""

from __future__ import annotations

from dataclasses import dataclass
import math

from hgv_control.models.longitudinal import LongitudinalParams, State, clamp


@dataclass
class BaselineController:
    h_gain: float = 1.5e-4
    gamma_gain: float = 1.15
    q_gain: float = 0.55
    v_gain: float = 7.0e-4
    trim_throttle: float = 0.62
    alpha_trim: float = math.radians(2.0)

    def command(
        self,
        state: State,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
    ) -> tuple[float, float]:
        gamma_cmd = clamp(self.h_gain * (altitude_ref - state.altitude), math.radians(-8.0), math.radians(8.0))
        alpha_cmd = self.alpha_trim + clamp(self.gamma_gain * (gamma_cmd - state.gamma), math.radians(-5.0), math.radians(5.0))
        theta_cmd = alpha_cmd + state.gamma
        delta_cmd = -0.9 * (theta_cmd - state.theta) + self.q_gain * state.pitch_rate
        delta_cmd = clamp(delta_cmd, -params.delta_limit, params.delta_limit)

        throttle_cmd = self.trim_throttle + self.v_gain * (velocity_ref - state.velocity)
        throttle_cmd = clamp(throttle_cmd, params.throttle_min, params.throttle_max)
        return delta_cmd, throttle_cmd

