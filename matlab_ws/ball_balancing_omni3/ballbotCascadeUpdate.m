function output = ballbotCascadeUpdate(estimate, command, ...
    previousVelocityIntegral, enable, previousYawBiasReady, ...
    biasDiagnostics, wheelRate, p)
%BALLBOTCASCADEUPDATE Cascaded tilt/ball-velocity wheel-torque control.
%   MiaPURE-style architecture (Xiao et al., arXiv:2304.02887): a 6-state
%   LQR owns [tilt(2); tiltRate(2); ballVelocity(2)] in the body yaw
%   frame, with commanded tilt produced by an outer velocity-error loop
%   and commanded ball velocity taken straight from the translation
%   command.  A bounded leaky PI on the ball-velocity error adds the
%   stiction/friction compensation the paper shows a model-only LQR
%   cannot provide.  Yaw keeps the gated common-mode channel.
%
%   Same I/O contract as ballbotLqiControllerUpdate so the model block
%   is unchanged: output = [wheelTorque(3); velocityIntegral(2); mode;
%   yawBiasReady].

roll = estimate(1);
pitch = estimate(2);
yaw = estimate(3);
bodyRate = estimate(4:6);
velocityWorld = estimate(7:8);
contactConfidence = estimate(14);
tiltMagnitude = hypot(roll, pitch);

yawBiasReady = ballbotYawBiasStartupGuard(previousYawBiasReady ~= 0, ...
    bodyRate(3), biasDiagnostics(2) ~= 0, p);

if ~enable || tiltMagnitude >= p.controller.fallenTilt
    mode = uint8(0);
elseif tiltMagnitude >= p.controller.recoveryTilt || ...
        contactConfidence < p.controller.minimumContactConfidence
    mode = uint8(2);
else
    mode = uint8(1);
end

rotationBodyFromWorld = [cos(yaw), sin(yaw); ...
    -sin(yaw), cos(yaw)];
velocityBody = rotationBodyFromWorld*velocityWorld;
commandVelocityBody = rotationBodyFromWorld*command(1:2);
commandSpeed = norm(commandVelocityBody);
if commandSpeed > p.controller.maxSpeed
    commandVelocityBody = commandVelocityBody* ...
        (p.controller.maxSpeed/commandSpeed);
end

if mode == 1
    velocityError = commandVelocityBody - ...
        rotationBodyFromWorld*[p.ball.radius*estimate(10); ...
        -p.ball.radius*estimate(9)];
    candidateIntegral = previousVelocityIntegral + ...
        p.controller.sampleTime*velocityError;
    integralLimit = p.controller.velocityIntegralLimit;
    candidateIntegral = min(max(candidateIntegral, -integralLimit), ...
        integralLimit);
else
    candidateIntegral = zeros(2, 1);
end

cc = p.controller.cascade;
yawRateCommand = min(max(command(3), -p.controller.maxYawRate), ...
    p.controller.maxYawRate);
if ~yawBiasReady && abs(command(3)) <= ...
        p.controller.yawBiasCommandBypassThreshold
    yawRateCommand = 0;
end

% Gyro-rate low-pass, same rationale as the torque-LQI path.
filteredBodyRate = bodyRate;
if cc.rateFilterTimeConstant > 0
    persistent bodyRateFilter
    if isempty(bodyRateFilter) || mode ~= 1
        bodyRateFilter = bodyRate;
    else
        wRate = p.controller.sampleTime/ ...
            (cc.rateFilterTimeConstant + p.controller.sampleTime);
        bodyRateFilter = bodyRateFilter + ...
            wRate*(bodyRate - bodyRateFilter);
    end
    filteredBodyRate = bodyRateFilter;
end

% Body-frame ball velocity: the load-bearing translation coordinate.
ballVelocityWorld = [p.ball.radius*estimate(10); ...
    -p.ball.radius*estimate(9)];
% Slip guard: when the ball spins without translating the two velocity
% estimates disagree; feeding the bogus rate into the velocity channel
% pumps the spin (observed startup-slip blowup, est(9:10) -> 30 rad/s).
% On slip, freeze the whole velocity path and keep pure tilt balance.
slipDetected = norm(ballVelocityWorld - velocityWorld) > ...
    cc.slipVelocityThreshold;
ballVelocityBody = rotationBodyFromWorld*ballVelocityWorld;

% Outer loop: ball-velocity error -> commanded lean.  Translation can
% only be produced through gravity (lean), so the demand is the
% commanded acceleration equivalent; slew-rate limiting keeps the ball
% inside the contact-friction tracking bandwidth, and the cap keeps the
% commanded lean inside the static hold cone with margin.
velocityErrorBody = commandVelocityBody - ballVelocityBody;
if slipDetected
    velocityErrorBody(:) = 0;
end
% RelPos centering: the ball escaping from under the chassis is the
% dominant divergence mode; a modest position term leans the chassis
% toward the offset so gravity re-centers the ball.  Kept weak --
% nonzero relPos is the correct acceleration equilibrium.
positionDemand = cc.positionGain*estimate(12:13);
positionDemand = min(max(positionDemand, -cc.positionDemandLimit), ...
    cc.positionDemandLimit);
accelerationDemand = cc.velocityLeanGain*velocityErrorBody + ...
    positionDemand;
leanLimit = cc.leanSetpointLimit;
accelerationDemand = min(max(accelerationDemand, -leanLimit*p.gravity), ...
    leanLimit*p.gravity);
persistent accelerationDemandSlew
if isempty(accelerationDemandSlew) || mode ~= 1
    accelerationDemandSlew = accelerationDemand;
else
    stepLimit = cc.leanRateLimit*p.gravity*p.controller.sampleTime;
    demandDelta = accelerationDemand - accelerationDemandSlew;
    demandDelta = min(max(demandDelta, -stepLimit), stepLimit);
    accelerationDemandSlew = accelerationDemandSlew + demandDelta;
end
accelerationDemand = accelerationDemandSlew;
commandedTilt = [accelerationDemand(1); -accelerationDemand(2)]/ ...
    p.gravity;

% Cone guard: tilt beyond the commanded lean gets no additional lean
% demand.
excessTilt = max(tiltMagnitude - leanLimit, 0);
fade = max(1 - excessTilt/cc.leanFadeTilt, 0);
commandedTilt = fade*commandedTilt;

% 6-state feedback: [pitch; -roll; q; -p; vBallBody(2)] with reference
% [commandedTilt; 0;0; commandVelocityBody].  The tilt sub-block keeps
% full authority from the first step; the ball-velocity sub-block and
% the PI are ramped in over the soft-start window because during the
% settle transient the ball slips/spins and the velocity channel pumps
% the spin up (observed: wheel-ball slip excited est(9:10) to ~30 rad/s
% and tipped the 0.5 kg build at t=0.48 s).
state = [pitch; -roll; filteredBodyRate(2); -filteredBodyRate(1); ...
    ballVelocityBody];
reference = [commandedTilt; 0; 0; commandVelocityBody];
tiltState = [state(1:4); zeros(2, 1)];
velocityState = state(5:6) - reference(5:6);
if slipDetected
    velocityState(:) = 0;
end
% Do not chase phantom speed: cap the velocity-state error so a slip
% transient cannot demand large counter-torque.
velocityState = min(max(velocityState, -p.controller.maxSpeed), ...
    p.controller.maxSpeed);
% The commanded lean is itself a disturbance path during startup: ball
% slip makes the velocity error huge, saturating the lean demand at the
% cap and driving a hard lean into the settle transient.
tiltReference = [startupRamp(commandedTilt, enable, mode, p); ...
    zeros(4, 1)];
rawWheelCommand = -cc.stateGain*(tiltState - tiltReference);
if cc.velocityStateFeedback
    rawWheelCommand = rawWheelCommand + startupRamp( ...
        -cc.stateGain(:, 5:6)*velocityState, enable, mode, p);
end

% Gravity-hold feedforward on the commanded lean, ramped in over the
% soft-start window together with the PI (both are disturbance paths;
% the balance feedback keeps full authority from the first step).
rawWheelCommand = rawWheelCommand + startupRamp( ...
    cc.leanHoldTorque*commandedTilt, enable, mode, p);

% Inner PI on the ball-velocity tracking error: compensates the
% unmodeled contact friction/stiction that a model-only LQR cannot
% reject.  Allocated through pinv(Bvel) so it produces pure ball
% acceleration (minimum-norm pattern; the tilt loop absorbs the
% residual).  Leaky + state-clamped = no windup through saturation.
if cc.ballVelocityKi ~= 0 || cc.ballVelocityKp ~= 0
    persistent ballVelIntegral
    if isempty(ballVelIntegral) || mode ~= 1
        ballVelIntegral = zeros(2, 1);
    else
        ballVelIntegral = ballVelIntegral + ...
            p.controller.sampleTime*velocityErrorBody;
        ballVelIntegral = ballVelIntegral* ...
            (1 - cc.ballVelocityLeak*p.controller.sampleTime);
        ballVelIntegral = min(max(ballVelIntegral, ...
            -cc.ballVelocityIntegralLimit), ...
            cc.ballVelocityIntegralLimit);
    end
    correctionAccel = cc.ballVelocityKp*velocityErrorBody + ...
        cc.ballVelocityKi*ballVelIntegral;
    rawWheelCommand = rawWheelCommand + startupRamp( ...
        cc.ballVelocityAllocation*correctionAccel, enable, mode, p);
end

% Common-mode removal: every non-yaw torque path is zero-sum by design,
% but the residual mean pumps the ball's vertical spin (omegaBz ran
% 0 -> 3.8 rad/s inside 0.5 s and dropped the contacts).  Subtract the
% mean from the balance/PI/FF sum; damping and yaw add after.
rawWheelCommand = rawWheelCommand - mean(rawWheelCommand);

% Wheel-rate damping: a wheel spinning faster than the kinematic rate
% implied by the MEASURED motion is slipping and its torque cannot
% reach the ball, so damp the excess rate.  The reference is the
% measured state, not the command -- using the command would recreate
% the legacy propulsion servo whose reaction torque was a primary
% destabilizer.  The deadzone keeps encoder-quantization flicker
% (~3.7 rad/s) out of the channel.
if cc.wheelRateDamping > 0
    kinematicWheelRate = p.controller.lqi. ...
        wheelSpeedFromPlanarVelocity*[velocityBody; bodyRate(3)];
    slipRate = wheelRate(:) - kinematicWheelRate;
    slipActive = sign(slipRate).* ...
        max(abs(slipRate) - cc.slipDeadzone, 0);
    % Strip the common-mode: a +z ball spin drags all wheels to +w, so
    % braking that component pumps the spin harder (reaction: -tau
    % adds +z ball accel).  Differential slip only; the ball-spin brake
    % below owns the common-mode channel.
    slipActive = slipActive - mean(slipActive);
    rawWheelCommand = rawWheelCommand - cc.wheelRateDamping*slipActive;
end

% Ball vertical-spin brake: +common-mode torque accelerates the ball
% toward -z (measured), so braking torque has the SAME sign as omegaBz.
% Acts inside the encoder deadzone where the wheel-rate channel cannot
% see the spin yet.
if cc.ballSpinBrakeGain > 0
    rawWheelCommand = rawWheelCommand + ...
        cc.ballSpinBrakeGain*estimate(11)*ones(3, 1);
end

% Yaw: common-mode torque, gated to stationary yaw commands only.
yawGate = cc.yawGate;
stationary = norm(velocityBody) < yawGate.maxSpeed && ...
    tiltMagnitude < yawGate.maxTilt && commandSpeed == 0;
yawActive = stationary && abs(command(3)) > yawGate.commandThreshold;
if yawActive && cc.yawRateGain ~= 0
    yawError = yawRateCommand - filteredBodyRate(3);
    rawWheelCommand = rawWheelCommand + ...
        cc.yawAllocation*cc.yawRateGain*yawError;
end

limit = min(p.driver.continuousTorqueLimit, ...
    p.controller.tractionLimitedWheelTorque);
if mode == 1
    % Per-wheel clipping of a zero-sum balance pattern leaves a nonzero
    % mean, and that residual common-mode torque pumps the ball's
    % vertical spin (observed: omegaBz 0 -> 1.3 rad/s within 0.7 s of a
    % clean settle, all-wheel negative duty).  Split the command into
    % the intentional common-mode part (spin brake + yaw) and the
    % zero-sum balance part, scale the balance part UNIFORMLY into the
    % headroom the common part leaves, then recombine -- uniform
    % scaling preserves zero-sum exactly.
    commonTorque = mean(rawWheelCommand);
    balanceTorque = rawWheelCommand - commonTorque;
    headroom = max(limit - abs(commonTorque), 0);
    peakBalance = max(abs(balanceTorque));
    if peakBalance > headroom
        balanceTorque = balanceTorque*(headroom/peakBalance);
    end
    wheelSpeedCommand = commonTorque + balanceTorque;
elseif mode == 2
    wheelSpeedCommand = min(max(-p.controller.recoveryGainScale* ...
        cc.stateGain*tiltState, -limit), limit);
elseif mode == 0
    wheelSpeedCommand = zeros(3, 1);
end

if mode == 1
    massEffective = p.rover.mass + p.ball.mass + ...
        p.ball.inertia(1, 1)/p.ball.radius^2;
    accelerationResidual = ...
        p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2).'* ...
        (wheelSpeedCommand - rawWheelCommand)/massEffective;
    candidateIntegral = candidateIntegral + p.controller.sampleTime* ...
        p.controller.lqi.antiWindupGain*accelerationResidual;
    integralLimit = p.controller.velocityIntegralLimit;
    nextVelocityIntegral = min(max(candidateIntegral, ...
        -integralLimit), integralLimit);
else
    nextVelocityIntegral = zeros(2, 1);
end
output = [wheelSpeedCommand; nextVelocityIntegral; double(mode); ...
    double(yawBiasReady)];
end

function scaled = startupRamp(value, enable, mode, p)
%STARTRUPRAMP Ramp a torque contribution in over startupTorqueRampTime.
if cc_startupRampDisabled(p)
    scaled = value;
    return
end
persistent rampClock
if isempty(rampClock) || ~enable || mode == 0
    rampClock = 0;
else
    rampClock = rampClock + p.controller.sampleTime;
end
scaled = value*min(1, rampClock/ ...
    p.controller.cascade.startupTorqueRampTime);
end

function tf = cc_startupRampDisabled(p)
tf = ~isfield(p.controller.cascade, "startupTorqueRampTime") || ...
    p.controller.cascade.startupTorqueRampTime <= 0;
end
