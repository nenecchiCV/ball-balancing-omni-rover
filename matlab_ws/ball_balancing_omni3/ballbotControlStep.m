function [wheelTorque, nextVelocityIntegral, mode, diagnostics] = ...
    ballbotControlStep(estimate, command, previousVelocityIntegral, ...
    enable, p)
%BALLBOTCONTROLSTEP Cascaded planar-velocity, balance, and yaw controller.
% Command is [vx_W; vy_W; yawRate_W]. Mode: 0=disabled/fallen,
% 1=balance, 2=recovery.

roll = estimate(1);
pitch = estimate(2);
yaw = estimate(3);
bodyRate = estimate(4:6);
velocityWorld = estimate(7:8);
contactConfidence = estimate(14);

tiltMagnitude = hypot(roll, pitch);
if ~enable || tiltMagnitude >= p.controller.fallenTilt
    mode = uint8(0);
elseif tiltMagnitude >= p.controller.recoveryTilt || ...
        contactConfidence < p.controller.minimumContactConfidence
    mode = uint8(2);
else
    mode = uint8(1);
end

commandVelocityWorld = command(1:2);
commandSpeed = norm(commandVelocityWorld);
if commandSpeed > p.controller.maxSpeed
    commandVelocityWorld = commandVelocityWorld* ...
        (p.controller.maxSpeed/commandSpeed);
end
commandYawRate = min(max(command(3), -p.controller.maxYawRate), ...
    p.controller.maxYawRate);

rotationBodyFromWorld = [cos(yaw), sin(yaw); ...
    -sin(yaw), cos(yaw)];
velocityBody = rotationBodyFromWorld*velocityWorld;
commandVelocityBody = rotationBodyFromWorld*commandVelocityWorld;
velocityError = commandVelocityBody - velocityBody;

if mode == 1
    candidateIntegral = p.controller.velocityIntegralLeak* ...
        previousVelocityIntegral + ...
        p.controller.sampleTime*velocityError;
    integralLimit = p.controller.velocityIntegralLimit;
    candidateIntegral = min(max(candidateIntegral, -integralLimit), ...
        integralLimit);
else
    candidateIntegral = zeros(2, 1);
    velocityError = -velocityBody;
end

accelerationCommand = p.controller.velocityKp.*velocityError + ...
    p.controller.velocityKi.*candidateIntegral;

% The velocity loop yields authority as tilt grows. This keeps the inner
% balance loop in charge near the recovery boundary instead of continuing
% to demand additional lean for velocity tracking.
if tiltMagnitude <= p.controller.tiltPriorityStart
    velocityAuthority = 1;
else
    velocityAuthority = max(0, ...
        (p.controller.recoveryTilt - tiltMagnitude)/ ...
        (p.controller.recoveryTilt - p.controller.tiltPriorityStart));
end
accelerationCommand = velocityAuthority*accelerationCommand;

% Once velocity has converged, command an upright attitude explicitly and
% bleed the integral state so that residual bias cannot hold a lean angle.
if norm(velocityError) <= p.controller.velocityConvergenceBand
    accelerationCommand = zeros(2, 1);
    candidateIntegral = p.controller.velocityIntegralLeak* ...
        candidateIntegral;
end
accelerationMagnitude = norm(accelerationCommand);
if accelerationMagnitude > p.controller.maxPlanarAcceleration
    accelerationCommand = accelerationCommand* ...
        (p.controller.maxPlanarAcceleration/accelerationMagnitude);
end

rollReference = -atan2(accelerationCommand(2), p.gravity);
pitchReference = atan2(accelerationCommand(1), p.gravity);
rollReference = min(max(rollReference, -p.controller.maxLean), ...
    p.controller.maxLean);
pitchReference = min(max(pitchReference, -p.controller.maxLean), ...
    p.controller.maxLean);

gainScale = 1.0;
if mode == 2
    rollReference = 0;
    pitchReference = 0;
    commandYawRate = 0;
    gainScale = p.controller.recoveryGainScale;
end

ballTorqueCommand = [-gainScale*(p.controller.balanceKp(1)* ...
    (roll - rollReference) + p.controller.balanceKd(1)*bodyRate(1)); ...
    -gainScale*(p.controller.balanceKp(2)*(pitch - pitchReference) + ...
    p.controller.balanceKd(2)*bodyRate(2)); ...
    p.controller.yawRateKp*(commandYawRate - bodyRate(3))];

if mode == 0
    ballTorqueCommand = zeros(3, 1);
end

[wheelTorque, achievedBallTorque, saturation] = ...
    ballbotTorqueAllocator(ballTorqueCommand, p);
if any(abs(saturation) > 1.0e-12)
    nextVelocityIntegral = previousVelocityIntegral;
else
    nextVelocityIntegral = candidateIntegral;
end

diagnostics = [rollReference; pitchReference; velocityError; ...
    ballTorqueCommand; achievedBallTorque; saturation];
end
