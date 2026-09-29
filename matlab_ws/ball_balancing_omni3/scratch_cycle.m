% Cycle verification run: move/stop/yaw-separated profile at slip-limit
% speed.  Optional overrides applied to ballbotParams after PreLoadFcn.
% Saves key signals to cycle_result_<tag>.mat.
function scratch_cycle(stopTime, tag, ov)
if nargin < 1, stopTime = 28; end
if nargin < 2, tag = 'def'; end
mdl = 'ball_balancing_omni3_multibody_lqi_rev1';
open_system(mdl);
% Never trust an inherited ballbotParams: pooled batch workers keep the
% base workspace between jobs, and a stale set can silently zero the
% command profile.  Regenerate deterministically, then apply overrides.
% Optional payloadMass positional override rescales mass/COM before the
% generic field overrides.
payloadMassArg = [];
if nargin >= 3 && isstruct(ov) && isfield(ov, 'payloadMassArg')
    payloadMassArg = ov.payloadMassArg;
    ov = ov.overrides;
end
ballbotParams = ballbotRev1Parameters(payloadMassArg);
if nargin >= 3 && ~isempty(ov)
    % ov: Nx2 cell array of {'dotted.field.path', value} overrides
    for i = 1:size(ov, 1)
        ballbotParams = setfieldNested(ballbotParams, ov{i, 1}, ...
            ov{i, 2});
    end
end
assignin('base', 'ballbotParams', ballbotParams);
set_param(mdl, 'StopTime', num2str(stopTime));
out = sim(mdl, 'SimulationMode', 'normal');

res = struct();
res.t = out.get('roverPose').Time;
res.roverPose = out.get('roverPose').Data;
res.ballPose = out.get('ballPose').Data;
res.est = out.get('estimatedState').Data;
res.duty = out.get('wheelDutyCommand').Data;
res.N = [out.get('wheel1NormalForce').Data, ...
    out.get('wheel2NormalForce').Data, ...
    out.get('wheel3NormalForce').Data];
res.mode = out.get('controllerMode').Data;
res.cmd = out.get('velocityCommand').Data;
res.tilt = hypot(res.roverPose(:,4), res.roverPose(:,5));
iT = find(res.tilt > 0.5, 1);
res.tTip = inf;
if ~isempty(iT), res.tTip = res.t(iT); end
save(['cycle_result_' tag '.mat'], 'res');
fprintf('tag=%s tEnd=%.1f tTip=%.2f maxTilt=%.4f endYaw=%.3f\n', ...
    tag, res.t(end), res.tTip, max(res.tilt), res.roverPose(end, 6));
end

function s = setfieldNested(s, dottedPath, value)
parts = strsplit(dottedPath, '.');
sub(1) = struct('type', '.', 'subs', parts{1});
for i = 2:numel(parts)
    sub(i) = struct('type', '.', 'subs', parts{i}); %#ok<AGROW>
end
s = subsasgn(s, sub, value);
end
