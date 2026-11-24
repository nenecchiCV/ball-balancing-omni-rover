function results = deadzoneAcceptance(configTag, scenarios)
%DEADZONEACCEPTANCE Rev2 deadzone acceptance matrix runner.
%   results = deadzoneAcceptance(configTag, scenarios) runs
%   ballbotBenchmark on the rev2 model for one deadzone
%   configuration across the scenario list (default hold/stepX/yaw/
%   cycle).  configTag:
%     'nodz'     comp OFF and plant deadzone zeroed  (baseline)
%     'dz_comp'  comp ON, plant dz = identified      (deployed)
%     'dz_nocomp' comp OFF, plant dz = identified    (uncompensated)
%   Results are appended to deadzone_acceptance_results.mat and also
%   returned.

if nargin < 1
    configTag = 'dz_comp';
end
if nargin < 2
    scenarios = {'hold', 'stepX', 'yaw', 'cycle'};
end
model = 'ball_balancing_omni3_multibody_lqi_rev2';
switch configTag
    case 'nodz'
        ov = {{'drive.deadzoneCompEnabled', false}, ...
            {'drive.deadzonePlantPos', [0, 0, 0]}, ...
            {'drive.deadzonePlantNeg', [0, 0, 0]}};
    case 'dz_comp'
        ov = {};
    case 'dz_nocomp'
        ov = {{'drive.deadzoneCompEnabled', false}};
    otherwise
        error('deadzoneAcceptance:unknownConfig', configTag);
end
outFile = 'deadzone_acceptance_results.mat';
results = struct();
if isfile(outFile)
    results = load(outFile).results;
end
for i = 1:numel(scenarios)
    tag = sprintf('rev2_%s_%s', configTag, scenarios{i});
    r = ballbotBenchmark(scenarios{i}, tag, ov, [], model);
    results.(matlab.lang.makeValidName(tag)) = r;
    save(outFile, 'results');
end
end
