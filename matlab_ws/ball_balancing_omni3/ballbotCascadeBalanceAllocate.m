function output = ballbotCascadeBalanceAllocate( ...
    estimate, leanReference, command, enable, previousYawReady, ...
    biasDiagnostics, p)
%BALLBOTCASCADEBALANCEALLOCATE Inner attitude PID and torque allocation.

roll = estimate(1);
pitch = estimate(2);
bodyRate = estimate(4:6);
contactConfidence = estimate(14);
tiltMagnitude = hypot(roll, pitch);

yawBiasReady = ballbotYawBiasStartupGuard(previousYawReady ~= 0, ...
    bodyRate(3), biasDiagnostics(2) ~= 0, p);
if ~enable || tiltMagnitude >= p.controller.fallenTilt
    mode = uint8(0);
elseif tiltMagnitude >= p.controller.recoveryTilt || ...
        contactConfidence < p.controller.minimumContactConfidence
    mode = uint8(2);
else
    mode = uint8(1);
end

if mode == 2
    leanReference = zeros(2, 1);
    gainScale = p.controller.recoveryGainScale;
else
    gainScale = 1.0;
end
attitudeError = [roll; pitch] - leanReference;
ballTorque = [-gainScale*(p.controller.balanceKp(1)*attitudeError(1) + ...
    p.controller.balanceKd(1)*bodyRate(1)); ...
    -gainScale*(p.controller.balanceKp(2)*attitudeError(2) + ...
    p.controller.balanceKd(2)*bodyRate(2)); 0];
if yawBiasReady || abs(command(3)) > ...
        p.controller.yawBiasCommandBypassThreshold
    ballTorque(3) = p.controller.yawRateKp*(command(3) - bodyRate(3));
end
if mode == 0
    ballTorque(:) = 0;
end
wheelTorque = ballbotTorqueAllocator(ballTorque, p);
output = [wheelTorque; double(mode); double(yawBiasReady)];
end
