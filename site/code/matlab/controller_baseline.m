function [delta_cmd, throttle_cmd] = controller_baseline(state, altitude_ref, velocity_ref, params)
%CONTROLLER_BASELINE Cascaded PID-style teaching controller.

h_gain = 1.5e-4;
gamma_gain = 1.15;
q_gain = 0.55;
v_gain = 7.0e-4;
trim_throttle = 0.62;
alpha_trim = deg2rad(2.0);

gamma_cmd = clamp_value(h_gain * (altitude_ref - state.altitude), deg2rad(-8.0), deg2rad(8.0));
alpha_cmd = alpha_trim + clamp_value(gamma_gain * (gamma_cmd - state.gamma), deg2rad(-5.0), deg2rad(5.0));
theta_cmd = alpha_cmd + state.gamma;
delta_cmd = -0.9 * (theta_cmd - state.theta) + q_gain * state.pitch_rate;
delta_cmd = clamp_value(delta_cmd, -params.delta_limit, params.delta_limit);

throttle_cmd = trim_throttle + v_gain * (velocity_ref - state.velocity);
throttle_cmd = clamp_value(throttle_cmd, params.throttle_min, params.throttle_max);
end
