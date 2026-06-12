this_dir = fileparts(mfilename('fullpath'));
code_dir = fileparts(this_dir);
addpath(code_dir);

[baseline_trace, baseline_metrics] = simulate_case('baseline', 5.0, 0.05);
[guarded_trace, guarded_metrics] = simulate_case('guarded', 5.0, 0.05);

assert(numel(baseline_trace.time) == 101);
assert(numel(guarded_trace.time) == 101);
assert(isfield(baseline_metrics, 'q_violation_integral'));
assert(isfield(guarded_metrics, 'pass'));
assert(baseline_metrics.q_max > 0.0);
assert(guarded_metrics.q_max > 0.0);
assert(baseline_metrics.alpha_max_rad < deg2rad(18.0));
assert(guarded_metrics.alpha_max_rad < deg2rad(18.0));

fprintf('MATLAB smoke test passed: baseline q_max=%.3f, guarded q_max=%.3f\n', ...
    baseline_metrics.q_max, guarded_metrics.q_max);
