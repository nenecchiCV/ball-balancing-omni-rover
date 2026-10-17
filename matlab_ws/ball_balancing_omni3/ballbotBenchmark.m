function result = ballbotBenchmark(scenario, tag, ov, sampleTime)
%BALLBOTBENCHMARK Run a benchmark scenario on the rev1 model and score
%   the G-01..G-11 specification metrics.
%   result = ballbotBenchmark(scenario, tag, ov)
%     scenario: 'hold' | 'stepX' | 'yaw' | 'cycle' | 'bias'
%     tag:      label printed in the summary line
%     ov:       cell of {dottedPath, value} parameter overrides applied
%               after the scenario preset (same convention as
%               scratch_move)
%     sampleTime: optional estimator/controller rate [s]; forwarded to
%               ballbotRev1Parameters BEFORE the scenario preset and ov
%               so every Ts-derived quantity is rebuilt consistently
%               (default [] = 5 ms / 200 Hz)
%
%   The returned struct carries every measured metric plus a pass flag
%   per applicable spec item; metrics whose signals are unavailable are
%   left NaN.  Runs are deterministic: parameters are regenerated and
%   interpreted MATLAB Fcn persistent state is cleared before sim.
%
%   Scenarios:
%     hold   - zero command, 5 s           -> G-01 attitude, G-08 limits
%     stepX  - 0->0.10 m/s world-X, 15 s   -> G-02 velocity settling
%     yaw    - 0->0.5 rad/s yaw, 10 s      -> G-03 yaw-rate settling
%     cycle  - 3x move/stop cycles, 52 s   -> contact envelope, G-04/05
%     bias   - +0.02 rad/s yaw gyro bias, 20 s hold -> G-09/10/11
%
%   All runs share the realistic plant (IMU noise + shaft friction from
%   ballbotRev1Parameters/ballbotRev1PlantAugment).

if nargin < 1, scenario = 'hold'; end
if nargin < 2, tag = scenario; end
if nargin < 3, ov = {}; end
if nargin < 4, sampleTime = []; end

model = 'ball_balancing_omni3_multibody_lqi_rev1';
ballbotRev1SimConfig(model);
% Overrides prefixed 'design.' are chassis-design knobs consumed BEFORE
% the geometry/gain derivations (contactLatitude, ball.radius, ...);
% they are stripped here and forwarded into the parameter build.
design = {};
keep = true(1, numel(ov));
for i = 1:numel(ov)
    pair = ov{i};
    if strncmp(pair{1}, 'design.', 7)
        design{end+1} = {pair{1}(8:end), pair{2}};
        keep(i) = false;
    end
end
ov = ov(keep);
p = ballbotRev1Parameters([], sampleTime, design);

stopTime = 5;
switch scenario
    case 'hold'
        p.command.cyclesEnabled = false;
        p.command.velocityWorld = [0; 0];
        p.command.yawRate = 0;
        stopTime = 5;
    case 'stepX'
        p.command.cyclesEnabled = false;
        p.command.velocityWorld = [0.10; 0];
        p.command.yawRate = 0;
        stopTime = 15;
    case 'yaw'
        p.command.cyclesEnabled = false;
        p.command.velocityWorld = [0; 0];
        p.command.yawRate = 0.5;
        stopTime = 10;
    case 'cycle'
        % defaults: 3x (12 s move + 4 s stop), profileVelocity [0.12;0]
        stopTime = p.command.cycleStart + ...
            p.command.cycleCount*(p.command.cycleMoveDuration + ...
            p.command.cycleStopDuration) + 1;
    case 'bias'
        p.command.cyclesEnabled = false;
        p.command.velocityWorld = [0; 0];
        p.command.yawRate = 0;
        p.simulation.imuYawBias = 0.02;
        stopTime = 20;
    otherwise
        error('ballbotBenchmark:unknownScenario', ...
            'unknown scenario %s', scenario);
end
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
% Re-apply the runtime plant augmentation AFTER overrides: ov can touch
% augment-consumed fields (payload.lateralOffset, friction flags) that
% the param build already consumed, and InitFcn will not re-run the
% augmentation when the workspace parameter set is already fresh.
ballbotRev1PlantAugment(model, p);
assignin('base', 'ballbotParams', p);
in = Simulink.SimulationInput(model);
in = in.setModelParameter('StopTime', num2str(stopTime));
in = in.setVariable('ballbotParams', p);
clear functions
out = sim(in);
assignin('base', 'dbgOut', out);

result = struct('scenario', scenario, 'tag', tag, ...
    'stopTime', stopTime, 'spec', struct(), 'data', struct());
s = result.spec;

% --- truth channels -------------------------------------------------
rp = out.get('roverPose');            % [x y z roll pitch yaw]
bp = out.get('ballPose');             % [x y z ...]
tilt = hypot(rp.Data(:, 4), rp.Data(:, 5));
yawTrue = unwrap(rp.Data(:, 6));
tR = rp.Time;
% Assembly planar speed: the ball center translates at rolling speed.
ballVel = zeros(numel(bp.Time), 2);
ballVel(2:end, :) = diff(bp.Data(:, 1:2))./diff(bp.Time);
ballVel = movmean(ballVel, 41);       % ~smoothing over solver steps
speedTrue = hypot(ballVel(:, 1), ballVel(:, 2));
yawRateTrue = zeros(numel(tR), 1);
yawRateTrue(2:end) = diff(yawTrue)./diff(tR);
yawRateTrue = movmean(yawRateTrue, 41);

% --- command ---------------------------------------------------------
vc = out.get('velocityCommand');      % [vx vy yawRate]
cmdSpeed = hypot(vc.Data(:, 1), vc.Data(:, 2));

% --- estimates --------------------------------------------------------
es = out.get('estimatedState');       % 14-wide estimate vector
tE = es.Time;
estTilt = hypot(es.Data(:, 1), es.Data(:, 2));
estVel = es.Data(:, 7:8);
contactConf = es.Data(:, 14);
estYawRateBody = es.Data(:, 6);

% --- actuation / contact ----------------------------------------------
wd = out.get('wheelDutyCommand');
cs = out.get('contactStatus');
n1 = out.get('wheel1NormalForce');
n2 = out.get('wheel2NormalForce');
n3 = out.get('wheel3NormalForce');
wdisp = out.get('wheelDisplacement');
wheelRateTrue = zeros(size(wdisp.Data));
wheelRateTrue(2:end, :) = diff(wdisp.Data)./diff(wdisp.Time);

% --- metrics -----------------------------------------------------------
s.G01_maxTiltDeg = rad2deg(max(tilt));
s.G01_endTiltDeg = rad2deg(tilt(end));
s.tipTime = NaN;
tipIdx = find(tilt > 0.35, 1);        % ~20 deg = past recoverable lean
if ~isempty(tipIdx), s.tipTime = tR(tipIdx); end

% G-02: settling error over the last 2 s where cmd is steady nonzero
s.G02_settleErr = NaN;
steadyMask = cmdSpeed > 0.05;
if any(steadyMask)
    lastCmdT = vc.Time(find(steadyMask, 1, 'last'));
    win = (bp.Time >= lastCmdT - 1.5) & (bp.Time <= lastCmdT);
    if any(win)
        cmdDir = vc.Data(find(steadyMask, 1, 'last'), 1:2);
        cmdDir = cmdDir/norm(cmdDir);
        vProj = ballVel(win, :)*cmdDir(:);
        cmdMag = norm(vc.Data(find(steadyMask, 1, 'last'), 1:2));
        s.G02_settleErr = mean(vProj) - cmdMag;
        s.G02_settleSpeed = mean(vProj);
    end
end

% G-03: yaw-rate settling error over the last 1.5 s of nonzero yaw cmd
s.G03_yawErr = NaN;
yawMask = abs(vc.Data(:, 3)) > 0.2;
if any(yawMask)
    lastYawT = vc.Time(find(yawMask, 1, 'last'));
    win = (tR >= lastYawT - 1.0) & (tR <= lastYawT);
    if any(win)
        s.G03_yawErr = mean(yawRateTrue(win)) - ...
            vc.Data(find(yawMask, 1, 'last'), 3);
    end
end

% G-04: three-wheel contact ratio (contactStatus cols = per-wheel flags)
wheelContact = cs.Data(:, 1:3) > 0.5;
if size(cs.Data, 2) >= 3
    s.G04_contactRatio = mean(all(wheelContact, 2));
end
% G-05: ball-ground separation time (z above nominal center height)
ballHeight = bp.Data(:, 3);
s.G05_separationTime = sum(diff(bp.Time).* ...
    double(ballHeight(1:end-1) > p.ball.radius + 1e-3));
s.G05_maxHeight = max(ballHeight);

% G-06/07: estimator errors (resample estimate onto truth grid)
estTiltOnTruth = interp1(tE, estTilt, tR, 'linear', 'extrap');
s.G06_maxEstTiltErrDeg = rad2deg(max(abs(estTiltOnTruth - tilt)));
trueVelOnEst = interp1(bp.Time, speedTrue, tE, 'linear', 'extrap');
estSpeed = hypot(estVel(:, 1), estVel(:, 2));
s.G07_rmsEstVelErr = sqrt(mean((estSpeed - trueVelOnEst).^2, ...
    'omitnan'));

% G-08: actuation limits
s.G08_maxDuty = max(abs(wd.Data(:)));
s.G08_maxWheelRate = max(abs(wheelRateTrue(:)));
s.G08_dutySaturated = any(abs(wd.Data(:)) >= 0.999);

% G-09..G-11: yaw gyro bias learning (bias scenario only)
s.G09_biasErrEnd = NaN;
s.G09_convTime = NaN;
s.G10_yawDriftDeg = NaN;
s.G11_biasMotionExcursion = NaN;
try
    yb = out.get('estimatedYawGyroBias');
    injected = p.simulation.imuYawBias;
    biasErr = injected - yb.Data;
    s.G09_biasErrEnd = biasErr(end);
    conv = find(abs(biasErr) < 0.002, 1);
    if ~isempty(conv), s.G09_convTime = yb.Time(conv); end
catch
end
% G-10: stationary yaw drift over the last 10 s -- only when the whole
% window is genuinely uncommanded (cmd speed and yaw rate ~0).
if tR(end) >= 10
    t0 = tR(end) - 10;
    cmdQuiet = all(abs(vc.Data(vc.Time >= t0, :)) < 1e-3, 'all');
    if cmdQuiet
        mask = tR >= t0;
        s.G10_yawDriftDeg = rad2deg(abs(yawTrue(find(mask, 1, 'last')) - ...
            yawTrue(find(mask, 1, 'first'))));
    end
end
result.spec = s;

% --- pass/fail ---------------------------------------------------------
pf = struct();
pf.G01 = s.G01_maxTiltDeg <= 1 && isnan(s.tipTime);
if ~isnan(s.G02_settleErr)
    pf.G02 = abs(s.G02_settleErr) <= 0.01;
end
if ~isnan(s.G03_yawErr)
    pf.G03 = abs(s.G03_yawErr) <= 0.05;
end
if isfield(s, 'G04_contactRatio')
    pf.G04 = s.G04_contactRatio >= 0.99;
end
pf.G05 = s.G05_separationTime == 0;
pf.G06 = s.G06_maxEstTiltErrDeg <= 0.5;
pf.G07 = s.G07_rmsEstVelErr <= 0.02;
pf.G08 = ~s.G08_dutySaturated;
if ~isnan(s.G09_biasErrEnd)
    pf.G09 = abs(s.G09_biasErrEnd) < 0.002;
end
if ~isnan(s.G10_yawDriftDeg)
    pf.G10 = s.G10_yawDriftDeg < 0.5;
end
result.pass = pf;

names = fieldnames(pf);
nPass = nnz(structfun(@(x) x, pf));
fprintf(['%-18s tilt=%.3fdeg tip=%s velErr=%s ' ...
    'contact=%.3f sep=%.2fs estTiltErr=%.2fdeg estVelErr=%.3f ' ...
    'duty=%.2f pass %d/%d\n'], ...
    tag, s.G01_maxTiltDeg, mat2str(s.tipTime), ...
    mat2str(s.G02_settleErr, 3), ...
    getOrNan(s, 'G04_contactRatio'), s.G05_separationTime, ...
    s.G06_maxEstTiltErrDeg, s.G07_rmsEstVelErr, s.G08_maxDuty, ...
    nPass, numel(names));
end

function v = getOrNan(s, f)
if isfield(s, f), v = s.(f); else, v = NaN; end
end
