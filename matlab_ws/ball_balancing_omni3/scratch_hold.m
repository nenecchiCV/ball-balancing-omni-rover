%SCRATCH_HOLD Zero-command upright hold: isolate startup-settle dynamics.
%   scratch_hold(tag, ov) - ov: cell of {dottedPath, value} overrides.
function scratch_hold(tag, ov)
if nargin < 1, tag = 'hold'; end
model = 'ball_balancing_omni3_multibody_lqi_rev1';
ballbotRev1SimConfig(model);
p = ballbotRev1Parameters;
p.command.cyclesEnabled = false;          % hold at zero command
p.command.velocityWorld = [0; 0];         % true zero command
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
in = in.setModelParameter('StopTime', '5');
in = in.setVariable('ballbotParams', p);
% See scratch_cycle.m: interpreted MATLAB Fcn persistents leak across
% sims in the same session; reset for reproducible runs.
clear functions
out = sim(in);
ti = out.SimulationMetadata.TimingInfo;

rp = out.get('roverPose'); tilt = hypot(rp.Data(:,4), rp.Data(:,5));
wd = out.get('wheelDutyCommand');
imu = out.get('imuMeasurement');
fprintf('%-22s exec=%5.1f  maxTilt=%.4f  tilt@1.5=%.4f  max|duty|=%.3f\n', ...
    tag, ti.ExecutionElapsedWallTime, max(tilt), tilt(end), max(abs(wd.Data(:))));
% tilt at a few checkpoints
for tc = [0.25 0.5 1.0 2.0 3.0 4.0]
    fprintf('   t=%4.2f tilt=%.4f gyroY=%+.3f\n', tc, ...
        interp1(rp.Time, tilt, tc), interp1(imu.Time, imu.Data(:,5), tc));
end
end
