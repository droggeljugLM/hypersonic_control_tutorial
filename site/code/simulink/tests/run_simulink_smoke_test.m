this_dir = fileparts(mfilename('fullpath'));
simulink_dir = fileparts(this_dir);
model_file = fullfile(simulink_dir, 'models', 'hgv_q_monitor.slx');
model_name = 'hgv_q_monitor';

open_system(model_file);
out = sim(model_name);

qbar = out.get('qbar');
q_margin = out.get('q_margin');
q_safe = out.get('q_safe');

qbar_value = qbar(end, end);
q_margin_value = q_margin(end, end);
q_safe_value = q_safe(end, end);

rho0 = 1.225;
altitude = 30000.0;
scale_height = 7200.0;
velocity = 1750.0;
q_limit = 70000.0;
expected_qbar = 0.5 * rho0 * exp(-altitude / scale_height) * velocity^2;

assert(abs(qbar_value - expected_qbar) < 1e-6);
assert(abs(q_margin_value - (q_limit - expected_qbar)) < 1e-6);
assert(q_safe_value == 1);

fprintf('Simulink smoke test passed: qbar=%.9f, q_margin=%.9f, q_safe=%d\n', ...
    qbar_value, q_margin_value, q_safe_value);
