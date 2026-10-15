function output = ballbotBallforceUpdate(estimate, command, ...
    previousVelocityIntegral, enable, previousYawBiasReady, ...
    biasDiagnostics, wheelRate, p)
%BALLBOTBALLFORCEUPDATE Ball-force servo controller for rev1.
%   Alternative control architecture: identical to the torque-input LQI
%   path in every rail (mode machine, filtered gyro/wheel-rate observer,
%   lean channel, tilt-gain scheduling, tilt integral, gravity-hold
%   feedforward, common-mode stripping, position spring, ball-spin
%   brake, yaw damper, slip damper, per-element clamp, anti-windup)
%   EXCEPT the translation channel: instead of the kinematic wheel-rate
%   P-servo (which cannot move the ball -- wheel rate is pinned to the
%   ball rolling rate by contact, so the servo saturates against an
%   unreachable rate target), a PI on the BALL velocity error produces a
%   planar force demand allocated to wheel torque through pinv(Bvel) --
%   the minimum-norm wheel-torque pattern that produces pure assembly
%   acceleration.  A Coulomb feedforward in the commanded wheel-rate
%   direction pre-loads the gear-shaft stiction the augmented plant adds.
%
%   Same I/O contract as ballbotLqiControllerUpdate:
%   output = [wheelTorque(3); velocityIntegral(2); mode; yawBiasReady].

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
% Follow-through braking (identical rule to the torque-LQI path).
td = p.controller.torqueDrive;
bf = p.controller.ballforce;
if td.stopFollowGain > 0
    ballVelocityWorld0 = [p.ball.radius*estimate(10); ...
        -p.ball.radius*estimate(9)];
    ballSpeed0 = norm(ballVelocityWorld0);
    if commandSpeed < td.stopFollowGain*ballSpeed0
        commandVelocityBody = rotationBodyFromWorld* ...
            (td.stopFollowGain*ballVelocityWorld0);
        commandSpeed = norm(commandVelocityBody);
    end
end

if mode == 1
    % Integrate the BALL velocity error, matching the lean channel.
    velocityError = commandVelocityBody - rotationBodyFromWorld* ...
        [p.ball.radius*estimate(10); -p.ball.radius*estimate(9)];
    candidateIntegral = previousVelocityIntegral + ...
        p.controller.sampleTime*velocityError;
    integralLimit = p.controller.velocityIntegralLimit;
    candidateIntegral = min(max(candidateIntegral, -integralLimit), ...
        integralLimit);
else
    candidateIntegral = zeros(2, 1);
end

state = [velocityBody; pitch; -roll; bodyRate(2); -bodyRate(1); ...
    bodyRate(3); wheelRate(:)];
yawRateCommand = min(max(command(3), -p.controller.maxYawRate), ...
    p.controller.maxYawRate);
if ~yawBiasReady && abs(command(3)) <= ...
        p.controller.yawBiasCommandBypassThreshold
    yawRateCommand = 0;
end

lqi = p.controller.torqueLqi;
filteredBodyRate = bodyRate;
if td.rateFilterTimeConstant > 0
    persistent bodyRateFilter
    if isempty(bodyRateFilter) || mode ~= 1
        bodyRateFilter = bodyRate;
    else
        wRate = p.controller.sampleTime/ ...
            (td.rateFilterTimeConstant + p.controller.sampleTime);
        bodyRateFilter = bodyRateFilter + ...
            wRate*(bodyRate - bodyRateFilter);
    end
    filteredBodyRate = bodyRateFilter;
end
state(5:7) = [filteredBodyRate(2); -filteredBodyRate(1); ...
    filteredBodyRate(3)];
filteredWheelRate = wheelRate(:);
if td.wheelRateObserverEnabled && td.wheelRateObserverGain > 0
    persistent wheelRateObs prevObserverTorque prevObserverKinRate
    if isempty(wheelRateObs) || mode ~= 1
        wheelRateObs = wheelRate(:);
        prevObserverTorque = zeros(3, 1);
        prevObserverKinRate = wheelRate(:);
    else
        wheelRateObs = wheelRateObs + ...
            (td.wheelRateObserverDriveGain*prevObserverTorque - ...
             td.wheelRateObserverCompliance* ...
             (wheelRateObs - prevObserverKinRate))* ...
            p.controller.sampleTime;
        wheelRateObs = wheelRateObs + td.wheelRateObserverGain* ...
            (wheelRate(:) - wheelRateObs);
    end
    filteredWheelRate = wheelRateObs;
elseif td.wheelRateFilterTimeConstant > 0
    persistent wheelRateFilter
    if isempty(wheelRateFilter) || mode ~= 1
        wheelRateFilter = wheelRate(:);
    else
        wWheel = p.controller.sampleTime/ ...
            (td.wheelRateFilterTimeConstant + p.controller.sampleTime);
        wheelRateFilter = wheelRateFilter + ...
            wWheel*(wheelRate(:) - wheelRateFilter);
    end
    filteredWheelRate = wheelRateFilter;
end
yawGate = td.yawGate;
stationary = norm(velocityBody) < yawGate.maxSpeed && ...
    tiltMagnitude < yawGate.maxTilt && commandSpeed == 0;
yawActive = stationary && abs(command(3)) > yawGate.commandThreshold;
if yawActive
    state(7) = state(7) - yawRateCommand;
else
    state(7) = 0;
end
if tiltMagnitude > p.controller.tiltPriorityStart
    candidateIntegral = zeros(2, 1);
end
% Slow leak for the ballforce integral: the force channel relies on the
% integral to supply the sustained ~0.02 N*m-per-wheel preload that
% breaks shaft stiction + rolling resistance.  The LQI leak (2/s) was
% sized for an unused integral; bled that fast it can never build the
% preload and a small translation error persists indefinitely.
candidateIntegral = candidateIntegral*(1 - bf.integralLeak* ...
    p.controller.sampleTime);
% The LQI path hard-disables this integrator (its integralGain is 0);
% the ballforce force channel owns it instead.
if isfield(bf, "integralEnabled") && ~bf.integralEnabled
    candidateIntegral = zeros(2, 1);
end
if isfield(td, "wheelRateFeedbackEnabled") && ...
        ~td.wheelRateFeedbackEnabled
    state(8:10) = 0;
end
% Outer loop, channel 1: velocity error -> lean setpoint (identical to
% the torque-LQI path).
velocityLeanGain = td.velocityLeanGain;
if velocityLeanGain ~= 0
    ballVelocityWorld = [p.ball.radius*estimate(10); ...
        -p.ball.radius*estimate(9)];
    ballVelocityBody = rotationBodyFromWorld*ballVelocityWorld;
    velocityErrorBody = commandVelocityBody - ballVelocityBody;
    accelerationDemand = velocityLeanGain*velocityErrorBody;
    if isfield(td, "accelFeedforwardGain") && ...
            td.accelFeedforwardGain ~= 0
        persistent prevCommandVelocity
        if isempty(prevCommandVelocity) || mode ~= 1
            prevCommandVelocity = commandVelocityBody;
        end
        commandAccelBody = (commandVelocityBody - ...
            prevCommandVelocity)/p.controller.sampleTime;
        prevCommandVelocity = commandVelocityBody;
        accelerationDemand = accelerationDemand + ...
            td.accelFeedforwardGain*commandAccelBody;
    end
    if isfield(td, "positionLeanGain")
        relativeVelocityBody = rotationBodyFromWorld* ...
            (estimate(7:8) - ballVelocityWorld);
        accelerationDemand = accelerationDemand - ...
            td.positionLeanGain*estimate(12:13) - ...
            td.positionLeanDampingGain*relativeVelocityBody;
    end
    leanLimit = td.leanSetpointLimit;
    accelerationDemand = min(max(accelerationDemand, -leanLimit* ...
        p.gravity), leanLimit*p.gravity);
    if isfield(td, "leanRateLimit") && td.leanRateLimit > 0
        persistent accelerationDemandSlew
        if isempty(accelerationDemandSlew) || mode ~= 1
            accelerationDemandSlew = accelerationDemand;
        else
            stepLimit = td.leanRateLimit* ...
                p.gravity*p.controller.sampleTime;
            demandDelta = accelerationDemand - accelerationDemandSlew;
            demandDelta = min(max(demandDelta, -stepLimit), stepLimit);
            accelerationDemandSlew = accelerationDemandSlew + ...
                demandDelta;
        end
        accelerationDemand = accelerationDemandSlew;
    end
    if isfield(td, "leanFadeTilt")
        excessTilt = max(tiltMagnitude - leanLimit, 0);
        fade = max(1 - excessTilt/td.leanFadeTilt, 0);
        accelerationDemand = fade*accelerationDemand;
    end
    state(3:4) = state(3:4) - ...
        [accelerationDemand(1); -accelerationDemand(2)]/p.gravity;
else
    accelerationDemand = zeros(2, 1);
end
% Tilt-gain scheduling (identical).
if isfield(td, "tiltStiffenGain")
    stiffenStart = td.leanSetpointLimit;
    stiffenBand = td.tiltStiffenTilt;
    boost = 1 + (td.tiltStiffenGain - 1)* ...
        min(max(tiltMagnitude - stiffenStart, 0)/stiffenBand, 1);
    state(3:6) = boost*state(3:6);
end
rawWheelCommand = -lqi.stateGain*state - ...
    lqi.integralGain*candidateIntegral;
% Bounded leaky tilt-error integrator (identical).
if isfield(td, "tiltIntegralGain") && td.tiltIntegralGain ~= 0
    persistent tiltIntegral
    if isempty(tiltIntegral) || mode ~= 1
        tiltIntegral = zeros(2, 1);
    else
        tiltIntegral = tiltIntegral + p.controller.sampleTime* ...
            state(3:4);
        tiltIntegral = tiltIntegral* ...
            (1 - td.tiltIntegralLeak*p.controller.sampleTime);
        tiltIntegral = min(max(tiltIntegral, -td.tiltIntegralLimit), ...
            td.tiltIntegralLimit);
    end
    rawWheelCommand = rawWheelCommand - td.tiltIntegralGain* ...
        lqi.stateGain(:, 3:4)*tiltIntegral;
end
if isfield(td, "leanHoldTorque") && velocityLeanGain ~= 0
    rawWheelCommand = rawWheelCommand + startupRamp( ...
        td.leanHoldTorque*[accelerationDemand(1); ...
        -accelerationDemand(2)]/p.gravity, enable, mode, p);
end
if ~yawActive
    rawWheelCommand = rawWheelCommand - mean(rawWheelCommand);
end
kinematicRate = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
    [velocityBody; bodyRate(3)];
if isfield(td, "positionGain") && td.positionGain ~= 0
    ballVelocityWorld = [p.ball.radius*estimate(10); ...
        -p.ball.radius*estimate(9)];
    relativeVelocityBody = rotationBodyFromWorld* ...
        (estimate(7:8) - ballVelocityWorld);
    rawWheelCommand = rawWheelCommand + ...
        p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2)* ...
        (td.positionGain*estimate(12:13) + ...
         td.positionDampingGain*relativeVelocityBody);
end
% Ball vertical-spin brake (identical).
ballYawGain = td.ballYawGain;
if ballYawGain ~= 0
    ballYawTorque = ballYawGain*estimate(11);
    if isfield(td, "ballYawTorqueLimit")
        ballYawTorque = min(max(ballYawTorque, ...
            -td.ballYawTorqueLimit), td.ballYawTorqueLimit);
    end
    rawWheelCommand = rawWheelCommand + ballYawTorque*ones(3, 1);
end
% Passive yaw damper during translation (identical).
if isfield(td, "yawDampingGain") && td.yawDampingGain ~= 0 && ...
        ~yawActive && commandSpeed > 0
    rawWheelCommand = rawWheelCommand - td.yawDampingGain* ...
        filteredBodyRate(3)*ones(3, 1);
end
slipRate = filteredWheelRate - kinematicRate;
if isfield(td, "filteredSlipDeadzone")
    slipDeadzone = td.filteredSlipDeadzone;
else
    slipDeadzone = td.slipDeadzone;
end
slipActive = sign(slipRate).*max(abs(slipRate) - slipDeadzone, 0);
rawWheelCommand = rawWheelCommand - td.slipDampingGain*slipActive;

% ===================== the new channel ================================
% Translation channel: ball-velocity PI -> planar force -> wheel torque.
%   Replaces the kinematic wheel-rate P-servo.  The ball's planar
%   dynamics are  vBall' = (F_contact + coupling)/mEff, and the minimum-
%   norm wheel torque producing a pure planar force is pinv(Bvel)*F.
%   Wheel rate is therefore no longer the servo error -- the request
%   scales with the actual ball velocity error, so the channel does not
%   stall behind the contact-pinned wheel rate.  A stiction feedforward
%   sign-on-commanded-rate pre-loads the gear-shaft Coulomb deadzone.
torqueLimit = min(p.driver.continuousTorqueLimit, ...
    p.controller.tractionLimitedWheelTorque);
dbgVelErr = zeros(2, 1);
forceDemand = zeros(2, 1);
propTorque = zeros(3, 1);
servoActive = false;
if mode == 1 && (bf.propulsionKp ~= 0 || bf.propulsionKi ~= 0 || ...
        bf.propulsionKd ~= 0)
    ballVelocityWorld = [p.ball.radius*estimate(10); ...
        -p.ball.radius*estimate(9)];
    ballVelocityBody = rotationBodyFromWorld*ballVelocityWorld;
    velocityErrorBody = commandVelocityBody - ballVelocityBody;
    % The ball-velocity estimate carries >0.15 m/s of flicker at rest;
    % driving it straight into wheel torque injects noise the tilt loop
    % must then fight.  Low-pass the error and gate the servo open only
    % on a real translation demand so a parked rover stays on the quiet
    % tilt loop alone.
    persistent filteredVelErr
    if isempty(filteredVelErr) || mode ~= 1
        filteredVelErr = velocityErrorBody;
    else
        wErr = p.controller.sampleTime/ ...
            (bf.velErrFilterTimeConstant + p.controller.sampleTime);
        filteredVelErr = filteredVelErr + wErr*(velocityErrorBody - ...
            filteredVelErr);
    end
    servoActive = commandSpeed > 0 || ...
        norm(filteredVelErr) >= bf.servoDeadzone;
    if ~servoActive
        filteredVelErr = zeros(2, 1);
    end
    dbgVelErr = filteredVelErr;
    relativeVelocityBody = rotationBodyFromWorld* ...
        (estimate(7:8) - ballVelocityWorld);
    % Command-acceleration feedforward: F = m_eff*a_cmd is the exact
    % traction demand of the commanded ramp; supplying it directly
    % counteracts the non-minimum-phase kick (the lean transient pushes
    % the ball backward ~0.03 m/s at move onset, which is the chaos seed
    % the feedback then has to recover from).
    forceDemand = bf.propulsionKp*filteredVelErr + ...
        servoActive*bf.propulsionKi*candidateIntegral - ...
        bf.propulsionKd*relativeVelocityBody;
    if bf.forceFeedforwardGain ~= 0 && ...
            exist('commandAccelBody', 'var')
        massEff = p.rover.mass + p.ball.mass + ...
            p.ball.inertia(1, 1)/p.ball.radius^2;
        ffAccel = min(max(bf.forceFeedforwardGain*massEff* ...
            commandAccelBody, -bf.forceFeedforwardLimit), ...
            bf.forceFeedforwardLimit);
        forceDemand = forceDemand + ffAccel;
    end
    forceDemand = min(max(forceDemand, -bf.propulsionForceLimit), ...
        bf.propulsionForceLimit);
    propTorque = bf.ballVelocityAllocation*forceDemand;
    if bf.stictionFeedforwardGain ~= 0
        wheelRateCommand = p.controller.lqi. ...
            wheelSpeedFromPlanarVelocity*[commandVelocityBody; 0];
        propTorque = propTorque + bf.stictionFeedforwardGain* ...
            p.motor.outputCoulombTorque*tanh( ...
            wheelRateCommand/bf.stictionRateWindow);
    end
    propTorque = min(max(propTorque, -bf.forceTorqueLimit), ...
        bf.forceTorqueLimit);
    headroom = max(torqueLimit - abs(rawWheelCommand), 0);
    headroom = max(headroom, bf.propulsionMinShare*torqueLimit);
    propTorque = sign(propTorque).*min(abs(propTorque), headroom);
    propTorque = startupRamp(propTorque, enable, mode, p);
    rawWheelCommand = rawWheelCommand + propTorque;
end
% Park brake: whenever the force servo is quiet (no translation demand,
% or force path disabled), fall back to the kinematic wheel-rate P-servo
% -- it brakes residual ball drift that the tilt loop cannot recenter
% (measured: without it the ball wanders ~0.03 m/s at stop until a
% wheel unloads ~15 s in).
if mode == 1 && ((bf.propulsionKp == 0 && bf.propulsionKi == 0 && ...
        bf.propulsionKd == 0) || ~servoActive)
    propulsionGain = td.propulsionGain;
    if propulsionGain ~= 0
        wheelRateCommand = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
            [commandVelocityBody; 0];
        if isfield(td, "propulsionRelPosGain")
            ballVelocityWorld = [p.ball.radius*estimate(10); ...
                -p.ball.radius*estimate(9)];
            relativeVelocityBody = rotationBodyFromWorld* ...
                (estimate(7:8) - ballVelocityWorld);
            relCorrection = td.propulsionRelPosGain*estimate(12:13) + ...
                td.propulsionRelVelGain*relativeVelocityBody;
            wheelRateCommand = wheelRateCommand + ...
                p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2)* ...
                relCorrection;
        end
        rateError = wheelRateCommand - filteredWheelRate;
        rateActive = sign(rateError).*max(abs(rateError) - ...
            slipDeadzone, 0);
        propTorque = min(max(propulsionGain*rateActive, ...
            -td.propulsionTorqueLimit), td.propulsionTorqueLimit);
        headroom = max(torqueLimit - abs(rawWheelCommand), 0);
        if isfield(td, "propulsionMinShare")
            headroom = max(headroom, td.propulsionMinShare*torqueLimit);
        end
        propTorque = sign(propTorque).*min(abs(propTorque), headroom);
        propTorque = startupRamp(propTorque, enable, mode, p);
        rawWheelCommand = rawWheelCommand + propTorque;
    end
end
% =====================================================================

limit = min(p.driver.continuousTorqueLimit, ...
    p.controller.tractionLimitedWheelTorque)*ones(3, 1);
wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
if isfield(td, "torqueSlewLimit") && td.torqueSlewLimit > 0 && mode == 1
    persistent prevTorqueCommand
    if isempty(prevTorqueCommand) || ~enable
        prevTorqueCommand = zeros(3, 1);
    end
    torqueStep = td.torqueSlewLimit*p.controller.sampleTime;
    wheelSpeedCommand = prevTorqueCommand + ...
        min(max(wheelSpeedCommand - prevTorqueCommand, ...
            -torqueStep), torqueStep);
    prevTorqueCommand = wheelSpeedCommand;
end
if mode == 2
    rawWheelCommand = -p.controller.recoveryGainScale* ...
        lqi.stateGain*state - td.slipDampingGain*slipActive;
    wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
elseif mode == 0
    wheelSpeedCommand(:) = 0;
end

if mode == 1 && td.wheelRateObserverEnabled
    prevObserverTorque = wheelSpeedCommand;
    prevObserverKinRate = kinematicRate;
end
if mode == 1
    massEffective = p.rover.mass + p.ball.mass + ...
        p.ball.inertia(1, 1)/p.ball.radius^2;
    accelerationResidual = p.controller.lqi. ...
        wheelSpeedFromPlanarVelocity(:, 1:2).'* ...
        (wheelSpeedCommand - rawWheelCommand)/massEffective;
    candidateIntegral = candidateIntegral + p.controller.sampleTime* ...
        p.controller.lqi.antiWindupGain*accelerationResidual;
    integralLimit = p.controller.velocityIntegralLimit;
    nextVelocityIntegral = min(max(candidateIntegral, -integralLimit), ...
        integralLimit);
else
    nextVelocityIntegral = zeros(2, 1);
end
if isfield(bf, "debugCapture") && bf.debugCapture
    persistent captureLog
    if isempty(captureLog)
        captureLog = zeros(0, 20);
    end
    captureLog(end+1, :) = [roll, pitch, velocityErrorBody.', ...
        dbgVelErr.', candidateIntegral.', forceDemand.', ...
        propTorque.', rawWheelCommand.', wheelSpeedCommand.', ...
        double(mode)];
    assignin('base', 'dbgBfCapture', captureLog);
end
output = [wheelSpeedCommand; nextVelocityIntegral; double(mode); ...
    double(yawBiasReady)];
end

function scaled = startupRamp(value, enable, mode, p)
%STARTRUPRAMP Ramp a torque contribution in over startupTorqueRampTime.
if ~isfield(p.controller.torqueDrive, "startupTorqueRampTime") || ...
        p.controller.torqueDrive.startupTorqueRampTime <= 0
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
    p.controller.torqueDrive.startupTorqueRampTime);
end
