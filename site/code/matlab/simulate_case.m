function [trace, metrics] = simulate_case(controller_name, duration, dt)
%SIMULATE_CASE Run one MATLAB teaching simulation.

if nargin < 1
    controller_name = 'guarded';
end
if nargin < 2
    duration = 45.0;
end
if nargin < 3
    dt = 0.02;
end

params = hgv_params();
state = make_initial_state();
steps = floor(duration / dt);
n = steps + 1;

trace.time = zeros(n, 1);
trace.velocity = zeros(n, 1);
trace.velocity_ref = zeros(n, 1);
trace.altitude = zeros(n, 1);
trace.altitude_ref = zeros(n, 1);
trace.gamma = zeros(n, 1);
trace.alpha = zeros(n, 1);
trace.theta = zeros(n, 1);
trace.pitch_rate = zeros(n, 1);
trace.elevator = zeros(n, 1);
trace.elevator_rate = zeros(n, 1);
trace.delta_cmd = zeros(n, 1);
trace.throttle_cmd = zeros(n, 1);
trace.qbar = zeros(n, 1);

for idx = 1:n
    time_s = (idx - 1) * dt;
    [altitude_ref, velocity_ref] = reference_profile(time_s);
    if strcmpi(controller_name, 'guarded')
        [altitude_cmd, velocity_cmd] = controller_guarded_ref(state, altitude_ref, velocity_ref, params);
    else
        altitude_cmd = altitude_ref;
        velocity_cmd = velocity_ref;
    end

    [delta_cmd, throttle_cmd] = controller_baseline(state, altitude_cmd, velocity_cmd, params);
    qbar = hgv_dynamic_pressure(state, params);

    trace.time(idx) = time_s;
    trace.velocity(idx) = state.velocity;
    trace.velocity_ref(idx) = velocity_ref;
    trace.altitude(idx) = state.altitude;
    trace.altitude_ref(idx) = altitude_ref;
    trace.gamma(idx) = state.gamma;
    trace.alpha(idx) = state.alpha;
    trace.theta(idx) = state.theta;
    trace.pitch_rate(idx) = state.pitch_rate;
    trace.elevator(idx) = state.elevator;
    trace.elevator_rate(idx) = state.elevator_rate;
    trace.delta_cmd(idx) = delta_cmd;
    trace.throttle_cmd(idx) = throttle_cmd;
    trace.qbar(idx) = qbar;

    state = hgv_rk4_step(state, delta_cmd, throttle_cmd, dt, params);
end

metrics = compute_metrics(trace, params, dt);
end
