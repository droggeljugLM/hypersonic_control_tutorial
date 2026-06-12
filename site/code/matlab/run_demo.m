function run_demo()
%RUN_DEMO Generate baseline and guarded MATLAB teaching traces.

this_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(this_dir, 'results');

[baseline_trace, baseline_metrics] = simulate_case('baseline');
[guarded_trace, guarded_metrics] = simulate_case('guarded');

write_trace_csv(baseline_trace, fullfile(results_dir, 'matlab_baseline_trace.csv'));
write_trace_csv(guarded_trace, fullfile(results_dir, 'matlab_guarded_trace.csv'));

print_metrics('baseline', baseline_metrics);
print_metrics('guarded', guarded_metrics);
end
