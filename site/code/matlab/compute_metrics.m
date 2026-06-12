function metrics = compute_metrics(trace, params, dt)
%COMPUTE_METRICS Compute pass/fail metrics from raw trace data.

h_error = trace.altitude - trace.altitude_ref;
v_error = trace.velocity - trace.velocity_ref;
q_values = trace.qbar;
delta_values = abs(trace.elevator);
delta_rate_values = abs(trace.elevator_rate);
alpha_values = abs(trace.alpha);
throttle_values = trace.throttle_cmd;

metrics.h_rms = sqrt(mean(h_error .^ 2));
metrics.v_rms = sqrt(mean(v_error .^ 2));
metrics.q_max = max(q_values);
metrics.q_margin_min = min(params.q_limit - q_values);
metrics.q_violation_integral = sum(max(0.0, q_values - params.q_limit)) * dt;
metrics.alpha_max_rad = max(alpha_values);
metrics.delta_max_rad = max(delta_values);
metrics.delta_rate_max_rad_s = max(delta_rate_values);
metrics.throttle_span = max(throttle_values) - min(throttle_values);
metrics.tracking_pass = metrics.h_rms <= 900.0 && metrics.v_rms <= 450.0;
metrics.q_pass = metrics.q_violation_integral <= 1.0;
metrics.alpha_pass = metrics.alpha_max_rad <= deg2rad(12.0);
metrics.delta_pass = metrics.delta_max_rad <= params.delta_limit + 1e-9;
metrics.delta_rate_pass = metrics.delta_rate_max_rad_s <= params.delta_rate_limit + 1e-9;
metrics.pass = metrics.tracking_pass && metrics.q_pass && metrics.alpha_pass ...
    && metrics.delta_pass && metrics.delta_rate_pass;
end
