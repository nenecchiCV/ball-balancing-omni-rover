function table_ = deadzoneSweep(cases)
%DEADZONESWEEP Deadzone-mismatch robustness sweep on the rev2 model.
%   The compensation stays at the identified firmware values while
%   the PLANT deadzone varies, reproducing unit-to-unit drift, wear
%   and direction asymmetry the TUNE sweep cannot track in real time.
%   Each case runs hold + stepX and records the benchmark metrics.
%
%   cases (optional) is a cell array of {label, ov} pairs; the
%   default set sweeps a uniform plant scale and two asymmetry
%   patterns.  Results append to deadzone_sweep_results.mat.

model = 'ball_balancing_omni3_multibody_lqi_rev2';
compPos = [0.1817, 0.1596, 0.1786];
compNeg = [0.1512, 0.1299, 0.1960];
if nargin < 1
    cases = {};
    for s = [0.5, 0.75, 1.25, 1.5, 2.0]
        ov = {{'drive.deadzonePlantPos', s*compPos}, ...
            {'drive.deadzonePlantNeg', s*compNeg}};
        cases{end+1} = {sprintf('scale%.2f', s), ov}; %#ok<AGROW>
    end
    cases{end+1} = {'wheel1x2', {{'drive.deadzonePlantPos', ...
        [2*compPos(1), compPos(2:3)]}, {'drive.deadzonePlantNeg', ...
        [2*compNeg(1), compNeg(2:3)]}}};
    cases{end+1} = {'dirSkew', {{'drive.deadzonePlantPos', ...
        1.5*compPos}, {'drive.deadzonePlantNeg', 0.5*compNeg}}};
end
outFile = 'deadzone_sweep_results.mat';
results = struct();
if isfile(outFile)
    results = load(outFile).results;
end
for c = 1:numel(cases)
    label = matlab.lang.makeValidName(cases{c}{1});
    ov = cases{c}{2};
    for scen = {'hold', 'stepX'}
        tag = sprintf('sw_%s_%s', label, scen{1});
        r = ballbotBenchmark(scen{1}, tag, ov, [], model);
        results.(tag) = r;
        save(outFile, 'results');
    end
end
table_ = results;
end
