function output = ballbotFullPlantHierarchyUpdate( ...
    estimate, command, previousVelocityIntegral, enable, ...
    previousYawBiasReady, biasDiagnostics, wheelRate, p)
%BALLBOTFULLPLANTHIERARCHYUPDATE Full-plant wheel-speed hierarchy.

roll = estimate(1);
pitch = estimate(2);
yaw = estimate(3);
bodyRate = estimate(4:6);
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

rotationBodyFromWorld = [cos(yaw), sin(yaw); -sin(yaw), cos(yaw)];
wheelVelocityMap = p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2);
velocityBody = pinv(wheelVelocityMap)*wheelRate(:);
commandVelocityBody = rotationBodyFromWorld*command(1:2);
commandVelocityBody = p.controller.fullplant.commandScale* ...
    commandVelocityBody;
commandNorm = norm(commandVelocityBody);
if commandNorm > p.controller.maxSpeed
    commandVelocityBody = commandVelocityBody*p.controller.maxSpeed/commandNorm;
end

velocityError = commandVelocityBody - velocityBody;
if mode == 1
    candidateIntegral = previousVelocityIntegral + ...
        p.controller.sampleTime*velocityError;
    candidateIntegral = min(max(candidateIntegral, ...
        -p.controller.velocityIntegralLimit), ...
        p.controller.velocityIntegralLimit);
else
    candidateIntegral = zeros(2, 1);
end

accelerationCommand = p.controller.fullplant.velocityKp.*velocityError + ...
    p.controller.fullplant.velocityKi.*candidateIntegral;
accelerationNorm = norm(accelerationCommand);
if accelerationNorm > p.controller.maxPlanarAcceleration
    accelerationCommand = accelerationCommand* ...
        p.controller.maxPlanarAcceleration/accelerationNorm;
end
tiltReference = [atan2(accelerationCommand(1), p.gravity); ...
    atan2(accelerationCommand(2), p.gravity)];
tiltReference = min(max(tiltReference, -p.controller.maxLean), ...
    p.controller.maxLean);

tiltState = [pitch; -roll];
tiltRate = [bodyRate(2); -bodyRate(1)];
if mode == 2
    tiltReference(:) = 0;
end
planarWheelVelocity = p.controller.fullplant.stabilizationSign*( ...
    p.controller.fullplant.tiltKp.*(tiltState - tiltReference) + ...
    p.controller.fullplant.tiltKd.*tiltRate);
planarWheelVelocity = planarWheelVelocity - ...
    p.controller.fullplant.commandFeedforward*commandVelocityBody;
if mode == 0
    planarWheelVelocity(:) = 0;
end

yawRateCommand = min(max(command(3), -p.controller.maxYawRate), ...
    p.controller.maxYawRate);
if ~yawBiasReady && abs(command(3)) <= ...
        p.controller.yawBiasCommandBypassThreshold
    yawRateCommand = 0;
end
wheelSpeedCommand = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
    [planarWheelVelocity; yawRateCommand];
if p.controller.fullplant.openLoopEnabled
    wheelSpeedCommand = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
        [commandVelocityBody; yawRateCommand];
end
limit = p.controller.lqi.maximumWheelSpeedCommand;
rawWheelSpeedCommand = wheelSpeedCommand;
wheelSpeedCommand = min(max(wheelSpeedCommand, -limit), limit);

if mode == 1 && any(abs(wheelSpeedCommand - rawWheelSpeedCommand) > 0)
    nextVelocityIntegral = previousVelocityIntegral;
else
    nextVelocityIntegral = candidateIntegral;
end
output = [wheelSpeedCommand; nextVelocityIntegral; double(mode); ...
    double(yawBiasReady)];
end
