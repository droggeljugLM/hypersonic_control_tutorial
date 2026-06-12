function next = hgv_rk4_step(state, delta_cmd, throttle_cmd, dt, params)
%HGV_RK4_STEP Advance one integration step.

k1 = hgv_derivatives(state, delta_cmd, throttle_cmd, params);
k2 = hgv_derivatives(hgv_add_state(state, k1, 0.5 * dt), delta_cmd, throttle_cmd, params);
k3 = hgv_derivatives(hgv_add_state(state, k2, 0.5 * dt), delta_cmd, throttle_cmd, params);
k4 = hgv_derivatives(hgv_add_state(state, k3, dt), delta_cmd, throttle_cmd, params);

next.velocity = state.velocity + dt * (k1.velocity + 2 * k2.velocity + 2 * k3.velocity + k4.velocity) / 6.0;
next.altitude = state.altitude + dt * (k1.altitude + 2 * k2.altitude + 2 * k3.altitude + k4.altitude) / 6.0;
next.gamma = state.gamma + dt * (k1.gamma + 2 * k2.gamma + 2 * k3.gamma + k4.gamma) / 6.0;
next.alpha = state.alpha + dt * (k1.alpha + 2 * k2.alpha + 2 * k3.alpha + k4.alpha) / 6.0;
next.pitch_rate = state.pitch_rate + dt * (k1.pitch_rate + 2 * k2.pitch_rate + 2 * k3.pitch_rate + k4.pitch_rate) / 6.0;
next.theta = state.theta + dt * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta) / 6.0;
next.elevator = state.elevator + dt * (k1.elevator + 2 * k2.elevator + 2 * k3.elevator + k4.elevator) / 6.0;
next.elevator_rate = state.elevator_rate + dt * (k1.elevator_rate + 2 * k2.elevator_rate + 2 * k3.elevator_rate + k4.elevator_rate) / 6.0;

next.velocity = max(100.0, next.velocity);
next.altitude = max(0.0, next.altitude);
next.elevator = clamp_value(next.elevator, -params.delta_limit, params.delta_limit);
next.elevator_rate = clamp_value(next.elevator_rate, -params.delta_rate_limit, params.delta_rate_limit);
next.alpha = clamp_value(next.alpha, deg2rad(-12.0), deg2rad(18.0));
end
