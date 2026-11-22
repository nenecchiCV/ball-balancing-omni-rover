%SCRATCH_ACCEPTANCE75 Regression acceptance for the 75 mm L2-L3 spacer.
%   Runs the standard scenario set (stepX/yaw/cycle + CG-20 mm hold;
%   'hold' baseline was run interactively) and saves each result to
%   acceptance75_results.mat as it completes so a long run can be
%   polled from outside.
function scratch_acceptance75
res = struct();
if isfile('acceptance75_results.mat')
    prev = load('acceptance75_results.mat');
    res = prev.results;
end

jobs = { ...
    'hold',  'acc75-hold',  {}; ...
    'stepX', 'acc75-stepX', {}; ...
    'yaw',   'acc75-yaw',   {}; ...
    'cycle', 'acc75-cycle', {}; ...
    'hold',  'acc75-cg20x', {{'fullplant.payload.lateralOffset', [0.02; 0]}}; ...
    };
for k = 1:size(jobs, 1)
    [scen, tag, ov] = jobs{k, :};
    key = matlab.lang.makeValidName(tag);
    if isfield(res, key)
        fprintf('skip %s (already done)\n', tag);
        continue
    end
    fprintf('=== %s (%s) ===\n', tag, scen);
    r = ballbotBenchmark(scen, tag, ov);
    res.(key) = r;
    results = res;
    save('acceptance75_results.mat', 'results');
end
fprintf('acceptance75 done\n');
end
