function [lift, drag, moment, thrust] = hgv_aero_forces(state, throttle, params)
%HGV_AERO_FORCES Compact aerodynamic and thrust model.

qbar = hgv_dynamic_pressure(state, params);
velocity = max(100.0, state.velocity);
alpha = state.alpha;
delta = state.elevator;

cl = params.cl0 + params.cl_alpha * alpha + params.cl_delta * delta;
cd = params.cd0 + params.cd_alpha2 * alpha * alpha;
cm = params.cm_alpha * alpha ...
    + params.cm_q * (params.ref_chord / (2.0 * velocity)) * state.pitch_rate ...
    + params.cm_delta * delta;

lift = qbar * params.ref_area * cl;
drag = qbar * params.ref_area * cd;
moment = qbar * params.ref_area * params.ref_chord * cm;
throttle = clamp_value(throttle, params.throttle_min, params.throttle_max);
thrust = params.thrust_max * throttle;
end
