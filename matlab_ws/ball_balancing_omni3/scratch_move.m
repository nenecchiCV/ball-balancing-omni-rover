%SCRATCH_MOVE Commanded move test: trapezoid to profile speed, watch tilt/relPos.
%   scratch_move(stopTime, ov) - ov: cell of {dottedPath, value} overrides.
function scratch_move(stopTime, ov)
if nargin < 1, stopTime = 14; end
model = 'ball_balancing_omni3_multibody_lqi_rev1';
ballbotRev1SimConfig(model);
p = ballbotRev1Parameters;
if nargin >= 2
    for i = 1:numel(ov)
        pair = ov{i};
        f = strsplit(pair{1}, '.');
        switch numel(f)
            case 1, p.(f{1}) = pair{2};
            case 2, p.(f{1}).(f{2}) = pair{2};
            case 3, p.(f{1}).(f{2}).(f{3}) = pair{2};
            case 4, p.(f{1}).(f{2}).(f{3}).(f{4}) = pair{2};
            otherwise, error('override path too deep: %s', pair{1});
        end
    end
end
assignin('base','ballbotParams',p);
in = Simulink.SimulationInput(model);
in = in.setModelParameter('StopTime', num2str(stopTime));
in = in.setVariable('ballbotParams', p);
out = sim(in);
assignin('base','dbgOut', out);
ti = out.SimulationMetadata.TimingInfo;

rp = out.get('roverPose'); tilt = hypot(rp.Data(:,4), rp.Data(:,5));
es = out.get('estimatedState');
vc = out.get('velocityCommand');
bp = out.get('ballPose');
wd = out.get('wheelDutyCommand');
tt = rp.Time;
fprintf('exec=%.1f  maxTilt=%.4f  tilt@end=%.4f\n', ti.ExecutionElapsedWallTime, max(tilt), tilt(end));
% command speed & ball speed at checkpoints
vB = hypot(p.ball.radius*es.Data(:,10), -p.ball.radius*es.Data(:,9));
relN = hypot(es.Data(:,12), es.Data(:,13));
fprintf('%6s %8s %8s %8s %8s %8s\n','t','tilt','cmd','vBall','relPos','x');
for tc = 0:0.5:min(stopTime, tt(end))
    k = interp1(tt,(1:numel(tt))',tc,'nearest','extrap');
    k = min(max(round(k),1),numel(tt));
    ek = interp1(es.Time,(1:numel(es.Time))',tc,'nearest','extrap');
    ek = min(max(round(ek),1),numel(es.Time));
    ck = interp1(vc.Time,(1:numel(vc.Time))',tc,'nearest','extrap');
    ck = min(max(round(ck),1),numel(vc.Time));
    bk = interp1(bp.Time,(1:numel(bp.Time))',tc,'nearest','extrap');
    bk = min(max(round(bk),1),numel(bp.Time));
    fprintf('%6.1f %8.4f %8.3f %8.3f %8.4f %8.3f\n', tc, tilt(k), ...
        vc.Data(ck,1), vB(ek), relN(ek), rp.Data(k,1));
end
% detail trace around engagement/move onset
fprintf('\n--- engage window 0.3-1.5 s ---\n');
esN = numel(es.Time);
for tc = 0.3:0.02:min(1.5, tt(end))
    k = interp1(tt,(1:numel(tt))',tc,'nearest','extrap');
    k = min(max(round(k),1),numel(tt));
    ek = interp1(es.Time,(1:esN)',tc,'nearest','extrap');
    ek = min(max(round(ek),1),esN);
    ck = interp1(vc.Time,(1:numel(vc.Time))',tc,'nearest','extrap');
    ck = min(max(round(ck),1),numel(vc.Time));
    fprintf('%6.2f t=%.4f cmd=%.3f vB=%.3f rel=%.4f oBz=%.3f\n', tc, ...
        tilt(k), vc.Data(ck,1), vB(ek), relN(ek), es.Data(ek,11));
end
end
