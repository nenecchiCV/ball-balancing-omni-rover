function output = ballbotLqiControllerUpdate( ...
    estimate, command, previousVelocityIntegral, enable, ...
    previousYawBiasReady, biasDiagnostics, wheelRate, p)
%BALLBOTLQICONTROLLERUPDATE Ten-state MIMO LQI wheel-speed controller.

if isfield(p.controller, "fullplant") && ...
        isfield(p.controller.fullplant, "enabled") && ...
        p.controller.fullplant.enabled
    output = ballbotFullPlantHierarchyUpdate(estimate, command, ...
        previousVelocityIntegral, enable, previousYawBiasReady, ...
        biasDiagnostics, wheelRate, p);
    return
end

roll = estimate(1);
pitch = estimate(2);
yaw = estimate(3);
bodyRate = estimate(4:6);
velocityWorld = estimate(7:8);
contactConfidence = estimate(14);
tiltMagnitude = hypot(roll, pitch);

yawBiasReady = ballbotYawBiasStartupGuard(previousYawBiasReady ~= 0, ...
    bodyRate(3), biasDiagnostics(2) ~= 0, p);
torqueDrive = isfield(p.controller, "torqueDrive") && ...
    isfield(p.controller.torqueDrive, "enabled") && ...
    p.controller.torqueDrive.enabled;

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
    velocityError = commandVelocityBody - velocityBody;
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
if torqueDrive
    % Torque-input LQI: the output is wheel torque [N*m], so the inner
    % wheel-speed servo disappears and tilt feedback acts at the
    % electrical time scale.  Yaw has no kinematic feedforward here; a
    % steady yaw rate needs zero torque, so the reference enters only
    % through the yaw-rate state error.
    lqi = p.controller.torqueLqi;
    debugCapture = isfield(p.controller.torqueDrive, "debugCapture") && ...
        p.controller.torqueDrive.debugCapture;
    if debugCapture
        captureInputs = [estimate(:); command(:); ...
            previousVelocityIntegral(:); enable; previousYawBiasReady; ...
            biasDiagnostics(:); wheelRate(:)];
    end
    if isfield(p.controller.torqueDrive, "openLoopTorque")
        % Plant-identification hook: bypass all feedback and apply a
        % fixed wheel-torque vector.
        output = [p.controller.torqueDrive.openLoopTorque(:); ...
            zeros(2, 1); double(mode); double(yawBiasReady)];
        return
    end
    state(7) = state(7) - yawRateCommand;
    if tiltMagnitude > p.controller.tiltPriorityStart
        candidateIntegral = zeros(2, 1);
    end
    % A torque-mode velocity integral is only weakly observable at rest;
    % estimator startup transients otherwise wind it up into a permanent
    % torque bias that creeps the ball out from under the chassis.
    % Leak it toward zero so only sustained velocity errors persist.
    candidateIntegral = candidateIntegral* ...
        (1 - p.controller.torqueDrive.integralLeak* ...
         p.controller.sampleTime);
    if isfield(p.controller.torqueDrive, "integralEnabled") && ...
            ~p.controller.torqueDrive.integralEnabled
        candidateIntegral = zeros(2, 1);
    end
    if isfield(p.controller.torqueDrive, "wheelRateFeedbackEnabled") && ...
            ~p.controller.torqueDrive.wheelRateFeedbackEnabled
        state(8:10) = 0;
    end
    rawWheelCommand = -lqi.stateGain*state - ...
        lqi.integralGain*candidateIntegral;
    % Traction control: a wheel that spins faster than the kinematic
    % rolling rate is slipping, so its torque cannot reach the ball.
    % Damping the slip rate keeps the contact engaged.  The deadzone
    % keeps encoder-quantization flicker (quantization/sampleTime ~3.7
    % rad/s) out of the damping channel.
    kinematicRate = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
        [velocityBody; bodyRate(3)];
    % Station keeping: relativePositionBody (rover - ball) feeds a small
    % torque through the wheel map so the chassis re-centers over the
    % ball.  Without it the ball creeps out from under the chassis --
    % the residual slow mode the tilt loop cannot see.
    if isfield(p.controller.torqueDrive, "positionGain")
        rawWheelCommand = rawWheelCommand - ...
            p.controller.torqueDrive.positionGain* ...
            p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2)* ...
            estimate(12:13);
    end
    slipRate = wheelRate(:) - kinematicRate;
    slipDeadzone = p.controller.torqueDrive.slipDeadzone;
    slipActive = sign(slipRate).*max(abs(slipRate) - slipDeadzone, 0);
    rawWheelCommand = rawWheelCommand - ...
        p.controller.torqueDrive.slipDampingGain*slipActive;
    limit = min(p.driver.continuousTorqueLimit, ...
        p.controller.tractionLimitedWheelTorque)*ones(3, 1);
    wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    if mode == 2
        rawWheelCommand = -p.controller.recoveryGainScale* ...
            lqi.stateGain*state - ...
            p.controller.torqueDrive.slipDampingGain*slipActive;
        wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    elseif mode == 0
        wheelSpeedCommand(:) = 0;
    end

    if mode == 1
        % Torque residual deflects planar acceleration; the equivalent
        % velocity correction leaks into the velocity integrators.
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
    if debugCapture
        persistent captureLog
        if isempty(captureLog)
            captureLog = zeros(0, 46);
        end
        captureLog(end+1, :) = [captureInputs.', state.', ...
            rawWheelCommand.', wheelSpeedCommand.', ...
            candidateIntegral.', double(mode)]; %#ok<AGROW>
        assignin('base', 'dbgLqiInputs', captureLog);
    end
    output = [wheelSpeedCommand; nextVelocityIntegral; double(mode); ...
        double(yawBiasReady)];
    return
end

% Planar tracking is introduced through the integral states so a velocity
% step cannot bypass the physical acceleration/lean limits. Yaw keeps its
% kinematic feedforward because it does not require a balancing lean.
feedforward = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
    [zeros(2, 1); yawRateCommand];
if tiltMagnitude > p.controller.tiltPriorityStart
    candidateIntegral = zeros(2, 1);
end
rawWheelCommand = feedforward - p.controller.lqi.stateGain*state - ...
    p.controller.lqi.trackingGainScale* ...
    p.controller.lqi.integralGain*candidateIntegral;
limit = p.motor.speedCommandLimit;
wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
if mode == 2
    rawWheelCommand = -p.controller.recoveryGainScale* ...
        p.controller.lqi.stateGain*state;
    wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
elseif mode == 0
    wheelSpeedCommand(:) = 0;
end

if mode == 1
    saturationCorrection = pinv( ...
        p.controller.lqi.wheelSpeedFromPlanarVelocity)* ...
        (wheelSpeedCommand - rawWheelCommand);
    candidateIntegral = candidateIntegral + p.controller.sampleTime* ...
        p.controller.lqi.antiWindupGain*saturationCorrection(1:2);
    integralLimit = p.controller.velocityIntegralLimit;
    nextVelocityIntegral = min(max(candidateIntegral, -integralLimit), ...
        integralLimit);
else
    nextVelocityIntegral = zeros(2, 1);
end
output = [wheelSpeedCommand; nextVelocityIntegral; double(mode); ...
    double(yawBiasReady)];
end
