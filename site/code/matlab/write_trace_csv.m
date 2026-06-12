function write_trace_csv(trace, output_path)
%WRITE_TRACE_CSV Write trace struct to CSV.

[output_dir, ~, ~] = fileparts(output_path);
if ~isempty(output_dir) && ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

data = struct2table(trace);
writetable(data, output_path);
end
