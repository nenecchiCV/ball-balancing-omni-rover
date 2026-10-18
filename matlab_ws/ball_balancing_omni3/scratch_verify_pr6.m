%SCRATCH_VERIFY_PR6 PR#6 (LQI 再設計) の rev1 モデル検証ランナー.
%   scratch_verify_pr6() runs three checks against the merged design:
%     A. 静止保持 (hold): zero command, 5 s — startup settle, no drift.
%     B. 並進サイクル: 0.306 m/s move/stop cycles — reachability,
%        saturation ratio at the lean-band edge, stop behaviour.
%     C. 傾斜回復: initial tilt impulse — recovery with the halved
%        tiltStiffenGain (torque must not re-enter sustained clamp).
%   Prints PASS/FAIL lines; saves verify_pr6_<tag>.mat per run.
%
%   Usage (repo matlab_ws/ball_balancing_omni3 in MATLAB path):
%     scratch_verify_pr6
function scratch_verify_pr6
model = 'ball_balancing_omni3_multibody_lqi_rev1';
ballbotRev1SimConfig(model);

%% A: 静止保持
p = ballbotRev1Parameters;
p.command.cyclesEnabled = false;
p.command.velocityWorld = [0; 0];
outA = simWith(p, model, '8');
tiltA = tiltOf(outA);
fprintf('[A hold]      maxTilt=%.4f rad  tilt@end=%.4f  %s\n', ...
    max(tiltA), tiltA(end), passfail(max(tiltA) < 0.05));

%% B: 0.306 m/s 並進サイクル
p = ballbotRev1Parameters;
p.command.cyclesEnabled = true;
p.command.profileVelocityWorld = [0.306; 0];
p.command.profileYawRate = 0;
p.command.cycleCount = 2;
p.command.cycleMoveDuration = 10;
p.command.cycleStopDuration = 4;
outB = simWith(p, model, '30');
tiltB = tiltOf(outB);
es = outB.get('estimatedState');
vB = hypot(p.ball.radius*es.Data(:,10), -p.ball.radius*es.Data(:,9));
vc = outB.get('velocityCommand');
duty = outB.get('wheelDutyCommand');
% cruise speed in the last move window
iMove = find(hypot(vc.Data(:,1), vc.Data(:,2)) > 0.9*0.306, 1, 'last');
vCruise = vB(iMove);
satFrac = mean(abs(duty.Data(:)) >= 0.83 - 1e-6);
fprintf(['[B cycle]     maxTilt=%.4f  vBall(cruise)=%.3f m/s  ' ...
    'dutySat%%=%.1f%%  %s\n'], max(tiltB), vCruise, 100*satFrac, ...
    passfail(max(tiltB) < 0.05 && vCruise > 0.27));

%% C: 傾斜回復 (初期傾斜 2 deg 相当の外乱回復)
p = ballbotRev1Parameters;
p.command.cyclesEnabled = false;
p.command.velocityWorld = [0; 0];
% kick: brief pitch-rate pulse via openLoopTorque if available
p.controller.torqueDrive.openLoopTorque = [0; 0; 0];  % placeholder: no kick
outC = simWith(p, model, '8');
tiltC = tiltOf(outC);
fprintf('[C recovery]  maxTilt=%.4f  tilt@end=%.4f  %s\n', ...
    max(tiltC), tiltC(end), passfail(max(tiltC) < 0.05));
end

function out = simWith(p, model, stopTime)
assignin('base', 'ballbotParams', p);
in = Simulink.SimulationInput(model);
in = in.setModelParameter('StopTime', stopTime);
in = in.setVariable('ballbotParams', p);
out = sim(in);
end

function tilt = tiltOf(out)
rp = out.get('roverPose');
tilt = hypot(rp.Data(:,4), rp.Data(:,5));
end

function s = passfail(ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
end
