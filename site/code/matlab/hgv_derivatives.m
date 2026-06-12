function xdot = hgv_derivatives(state, delta_cmd, throttle_cmd, params)
%HGV_DERIVATIVES Longitudinal teaching-model dynamics.

delta_cmd = clamp_value(delta_cmd, -params.delta_limit, params.delta_limit);
throttle_cmd = clamp_value(throttle_cmd, params.throttle_min, params.throttle_max);

[lift, drag, moment, thrust] = hgv_aero_forces(state, throttle_cmd, params);
velocity = max(100.0, state.velocity);

delta_accel = params.actuator_wn * params.actuator_wn * (delta_cmd - state.elevator) ...
    - 2.0 * params.actuator_zeta * params.actuator_wn * state.elevator_rate;
if abs(state.elevator_rate) >= params.delta_rate_limit && state.elevator_rate * delta_accel > 0.0
    delta_accel = 0.0;
end

v_dot = (thrust * cos(state.alpha) - drag) / params.mass ...
    - params.gravity * sin(state.gamma);
gamma_dot = (thrust * sin(state.alpha) + lift) / (params.mass * velocity) ...
    - params.gravity * cos(state.gamma) / velocity;
h_dot = state.velocity * sin(state.gamma);
q_dot = moment / params.iy;
theta_dot = state.pitch_rate;
alpha_dot = theta_dot - gamma_dot;

xdot.velocity = v_dot;
xdot.altitude = h_dot;
xdot.gamma = gamma_dot;
xdot.alpha = alpha_dot;
xdot.pitch_rate = q_dot;
xdot.theta = theta_dot;
xdot.elevator = state.elevator_rate;
xdot.elevator_rate = delta_accel;
end
