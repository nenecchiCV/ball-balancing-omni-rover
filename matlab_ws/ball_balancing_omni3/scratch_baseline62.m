%SCRATCH_BASELINE62 Baseline regression with the pre-change (62 mm spacer)
%   mass model.  Run while ballbotParameters.m is temporarily at HEAD so
%   the acc75 results can be compared gate-by-gate against the previous
%   acceptance state.  Saves each result to baseline62_results.mat.
function scratch_baseline62
res = struct();
if isfile('baseline62_results.mat')
    prev = load('baseline62_results.mat');
    res = prev.results;
end

jobs = { ...
    'hold',  'bl62-hold',  {}; ...
    'stepX', 'bl62-stepX', {}; ...
    'yaw',   'bl62-yaw',   {}; ...
    'cycle', 'bl62-cycle', {}; ...
    'hold',  'bl62-cg20x', {{'fullplant.payload.lateralOffset', [0.02; 0]}}; ...
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
    save('baseline62_results.mat', 'results');
end
fprintf('baseline62 done\n');
end
