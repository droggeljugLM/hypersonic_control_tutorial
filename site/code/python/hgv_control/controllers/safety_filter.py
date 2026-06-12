"""Reference-level dynamic-pressure guard for teaching examples."""

from __future__ import annotations

from hgv_control.models.longitudinal import LongitudinalParams, State, dynamic_pressure


def guarded_reference(
    state: State,
    altitude_ref: float,
    velocity_ref: float,
    params: LongitudinalParams,
) -> tuple[float, float]:
    """Modify references when dynamic pressure approaches the warning limit."""
    qbar = dynamic_pressure(state, params)
    if qbar <= params.q_warning:
        return altitude_ref, velocity_ref

    severity = min(1.0, (qbar - params.q_warning) / max(1.0, params.q_limit - params.q_warning))
    altitude_guard = altitude_ref + severity * 7000.0
    velocity_guard = velocity_ref - severity * 500.0
    return altitude_guard, max(900.0, velocity_guard)
