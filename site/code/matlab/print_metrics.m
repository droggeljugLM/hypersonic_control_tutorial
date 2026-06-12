function print_metrics(name, metrics)
%PRINT_METRICS Print a compact metric summary.

fprintf('%s: pass=%d, h_rms=%.3f, v_rms=%.3f, q_max=%.3f, q_violation_integral=%.3f\n', ...
    name, metrics.pass, metrics.h_rms, metrics.v_rms, metrics.q_max, metrics.q_violation_integral);
end
