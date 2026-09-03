function output = ballbotPidControllerUpdate( ...
    estimate, command, previousTiltIntegral, enable, ...
    previousYawBiasReady, biasDiagnostics, p)
%BALLBOTPIDCONTROLLERUPDATE Upright PID plus direct velocity torque control.
% The controller holds roll and pitch at zero. Planar velocity errors add
% wheel-driving torque directly; they are not converted to lean references.

rollPitch = estimate(1:2);
bodyRate = estimate(4:6);
yaw = estimate(3);
velocityWorld = estimate(7:8);
tiltMagnitude = norm(rollPitch);
active = enable ~= 0 && tiltMagnitude < p.controller.fallenTilt;
if active
    mode = uint8(1);
    tiltError = -rollPitch;
    candidateIntegral = previousTiltIntegral + ...
        p.controller.sampleTime*tiltError;
    limit = p.controller.pidTiltIntegralLimit;
    candidateIntegral = min(max(candidateIntegral, -limit), limit);
else
    mode = uint8(0);
    candidateIntegral = zeros(2, 1);
end

commandVelocityWorld = command(1:2);
commandSpeed = norm(commandVelocityWorld);
if commandSpeed > p.controller.maxSpeed
    commandVelocityWorld = commandVelocityWorld* ...
        (p.controller.maxSpeed/commandSpeed);
end
rotationBodyFromWorld = [cos(yaw), sin(yaw); -sin(yaw), cos(yaw)];
velocityErrorBody = rotationBodyFromWorld* ...
    (commandVelocityWorld - velocityWorld);

tiltTorque = p.controller.pidTiltKp.*(-rollPitch) + ...
    p.controller.pidTiltKi.*candidateIntegral + ...
    p.controller.pidTiltKd.*(-bodyRate(1:2));
if tiltMagnitude <= p.controller.tiltPriorityStart
    velocityAuthority = 1;
else
    velocityAuthority = max(0, ...
        (p.controller.recoveryTilt - tiltMagnitude)/ ...
        (p.controller.recoveryTilt - p.controller.tiltPriorityStart));
end
velocityTorque = velocityAuthority* ...
    [-p.controller.pidVelocityKp(2)*velocityErrorBody(2); ...
    p.controller.pidVelocityKp(1)*velocityErrorBody(1)];

yawBiasReady = ballbotYawBiasStartupGuard(previousYawBiasReady ~= 0, ...
    estimate(6), biasDiagnostics(2) ~= 0, p);
yawEnabled = yawBiasReady || ...
    abs(command(3)) > p.controller.yawBiasCommandBypassThreshold;
if yawEnabled
    yawTorque = p.controller.pidYawRateKp* ...
        (min(max(command(3), -p.controller.maxYawRate), ...
        p.controller.maxYawRate) - bodyRate(3));
else
    yawTorque = 0;
end

requestedBallTorque = [tiltTorque + velocityTorque; yawTorque];
if ~active
    requestedBallTorque = zeros(3, 1);
end
[wheelTorque, ~, saturation] = ballbotTorqueAllocator( ...
    requestedBallTorque, p);
if any(abs(saturation) > 1.0e-12)
    nextTiltIntegral = previousTiltIntegral;
else
    nextTiltIntegral = candidateIntegral;
end

output = [wheelTorque; nextTiltIntegral; double(mode); ...
    double(yawBiasReady)];
end
