"""Teaching CBF/BLF-style dynamic-pressure safety layer.

This module keeps the implementation deliberately small. It is a command
filter wrapped around a nominal tracker, not a formal CLF-CBF-QP solver.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Protocol

from hgv_control.controllers.pid import BaselineController
from hgv_control.models.longitudinal import (
    LongitudinalParams,
    State,
    atmosphere_density,
    clamp,
    derivatives,
    dynamic_pressure,
)


class TrackingController(Protocol):
    def command(
        self,
        state: State,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
    ) -> tuple[float, float]:
        ...


def dynamic_pressure_rate(
    state: State,
    delta_cmd: float,
    throttle_cmd: float,
    params: LongitudinalParams,
) -> float:
    """Approximate qbar rate under the current command."""
    qbar = dynamic_pressure(state, params)
    rho = atmosphere_density(state.altitude, params)
    state_dot = derivatives(state, delta_cmd, throttle_cmd, params)
    return -(qbar / params.scale_height) * state_dot.altitude + rho * state.velocity * state_dot.velocity


@dataclass
class BarrierController:
    """Low-computation dynamic-pressure barrier layer.

    The nominal controller handles tracking. This wrapper monitors
    h_q = q_limit - qbar and grows a correction as qbar approaches q_limit.
    The correction cuts throttle and biases elevator in the simplified
    model's unload direction.
    """

    nominal: TrackingController | None = None
    cbf_gain: float = 0.35
    blf_scale: float = 6.0
    qdot_scale: float = 1600.0
    margin_floor: float = 250.0
    max_throttle_cut: float = 0.95
    max_elevator_relief: float = math.radians(11.0)

    def _nominal_controller(self) -> TrackingController:
        return self.nominal if self.nominal is not None else BaselineController()

    def barrier_severity(
        self,
        state: State,
        delta_cmd: float,
        throttle_cmd: float,
        params: LongitudinalParams,
    ) -> float:
        qbar = dynamic_pressure(state, params)
        qdot = dynamic_pressure_rate(state, delta_cmd, throttle_cmd, params)
        h_q = params.q_limit - qbar
        warning_margin = max(1.0, params.q_limit - params.q_warning)

        if qbar <= params.q_warning and qdot <= self.cbf_gain * h_q:
            return 0.0

        distance_to_limit = max(h_q, self.margin_floor)
        blf_like_growth = max(0.0, warning_margin / distance_to_limit - 1.0)
        blf_severity = clamp(blf_like_growth / self.blf_scale, 0.0, 1.0)

        cbf_excess = qdot - self.cbf_gain * h_q
        cbf_severity = clamp(cbf_excess / self.qdot_scale, 0.0, 1.0)

        if h_q < 0.0:
            violation_severity = clamp((-h_q) / warning_margin, 0.0, 1.0)
            cbf_severity = max(cbf_severity, violation_severity)

        return max(blf_severity, cbf_severity)

    def command(
        self,
        state: State,
        altitude_ref: float,
        velocity_ref: float,
        params: LongitudinalParams,
    ) -> tuple[float, float]:
        delta_cmd, throttle_cmd = self._nominal_controller().command(state, altitude_ref, velocity_ref, params)
        severity = self.barrier_severity(state, delta_cmd, throttle_cmd, params)
        if severity <= 0.0:
            return delta_cmd, throttle_cmd

        safe_throttle = clamp(
            throttle_cmd - self.max_throttle_cut * severity,
            params.throttle_min,
            params.throttle_max,
        )
        safe_delta = clamp(
            delta_cmd - self.max_elevator_relief * severity,
            -params.delta_limit,
            params.delta_limit,
        )
        return safe_delta, safe_throttle
