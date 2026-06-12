this_dir = fileparts(mfilename('fullpath'));
simulink_dir = fileparts(this_dir);
model_file = fullfile(simulink_dir, 'models', 'hgv_longitudinal_closed_loop.slx');
model_name = 'hgv_longitudinal_closed_loop';

open_system(model_file);
out = sim(model_name);

state_arr = out.get('sim_state');
q_values = out.get('qbar');
q_violation_values = out.get('q_violation');
h_ref = out.get('altitude_ref');
v_ref = out.get('velocity_ref');
delta_cmd = out.get('delta_cmd');
throttle_cmd = out.get('throttle_cmd');

state_values = squeeze(state_arr);
if size(state_values, 1) == 8
    state_values = state_values.';
end

q_values = q_values(:);
q_violation_values = q_violation_values(:);
h_ref = h_ref(:);
v_ref = v_ref(:);
delta_cmd = delta_cmd(:);
throttle_cmd = throttle_cmd(:);

n = min([size(state_values, 1), numel(h_ref), numel(v_ref), ...
    numel(q_values), numel(q_violation_values)]);

assert(n > 100);
assert(size(state_values, 2) == 8);
assert(all(isfinite(state_values(1:n, :)), 'all'));
assert(all(isfinite(q_values(1:n))));
assert(all(isfinite(delta_cmd)));
assert(all(isfinite(throttle_cmd)));

h_values = state_values(1:n, 2);
v_values = state_values(1:n, 1);
h_rms = sqrt(mean((h_values - h_ref(1:n)).^2));
v_rms = sqrt(mean((v_values - v_ref(1:n)).^2));
q_violation_integral = sum(q_violation_values(1:n)) * 0.02;
q_max = max(q_values(1:n));
pass = h_rms <= 900.0 && v_rms <= 450.0 && q_violation_integral <= 1.0;

assert(h_rms <= 900.0);
assert(v_rms <= 450.0);
assert(q_max < 70000.0);
assert(q_violation_integral <= 1.0);
assert(pass);

fprintf(['Simulink closed-loop smoke test passed: pass=%d, h_rms=%.3f, ', ...
    'v_rms=%.3f, q_max=%.3f, q_violation_integral=%.3f\n'], ...
    pass, h_rms, v_rms, q_max, q_violation_integral);
