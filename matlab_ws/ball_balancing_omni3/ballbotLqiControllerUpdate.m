function output = ballbotLqiControllerUpdate( ...
    estimate, command, previousVelocityIntegral, enable, ...
    previousYawBiasReady, biasDiagnostics, ~, p)
%BALLBOTLQICONTROLLERUPDATE Planar LQI with wheel-torque command output.

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

% The two identical planar designs use alpha=[pitch;-roll] and
% alphaDot=[q;-p]. Their generalized torques map back as [tauX;tauY].
tilt = [pitch; -roll];
tiltRate = [bodyRate(2); -bodyRate(1)];
generalizedTorque = zeros(2, 1);
for axis = 1:2
    % lqi uses the output-minus-reference integrator convention. The
    % surrounding model stores command-minus-output, so the integral sign
    % is reversed here. The velocity term is written as a tracking error
    % explicitly to avoid embedding a nonzero equilibrium in the state.
    generalizedTorque(axis) = p.controller.lqi.stateGain(1)* ...
        (velocityBody(axis) - commandVelocityBody(axis)) - ...
        p.controller.lqi.stateGain(2)*tilt(axis) - ...
        p.controller.lqi.stateGain(3)*tiltRate(axis) + ...
        p.controller.lqi.integralGain*candidateIntegral(axis);
end
ballTorque = [-generalizedTorque(2); generalizedTorque(1); 0];

if yawBiasReady || abs(command(3)) > ...
        p.controller.yawBiasCommandBypassThreshold
    yawRateCommand = min(max(command(3), -p.controller.maxYawRate), ...
        p.controller.maxYawRate);
    ballTorque(3) = p.controller.yawRateKp* ...
        (yawRateCommand - bodyRate(3));
end
if mode == 2
    gainScale = p.controller.recoveryGainScale;
    ballTorque(1:2) = [-gainScale*(p.controller.balanceKp(1)*roll + ...
        p.controller.balanceKd(1)*bodyRate(1)); ...
        -gainScale*(p.controller.balanceKp(2)*pitch + ...
        p.controller.balanceKd(2)*bodyRate(2))];
elseif mode == 0
    ballTorque(:) = 0;
end

[wheelTorqueCommand, ~, torqueSaturation] = ...
    ballbotTorqueAllocator(ballTorque, p);
if mode == 0
    wheelTorqueCommand(:) = 0;
end
if any(abs(torqueSaturation) > 1.0e-12)
    nextVelocityIntegral = previousVelocityIntegral;
else
    nextVelocityIntegral = candidateIntegral;
end
output = [wheelTorqueCommand; nextVelocityIntegral; double(mode); ...
    double(yawBiasReady)];
end
