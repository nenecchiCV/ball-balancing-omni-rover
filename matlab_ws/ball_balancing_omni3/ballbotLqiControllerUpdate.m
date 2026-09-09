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
