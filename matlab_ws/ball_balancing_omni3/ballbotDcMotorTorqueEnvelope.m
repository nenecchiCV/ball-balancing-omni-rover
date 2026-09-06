function limitedTorque = ballbotDcMotorTorqueEnvelope( ...
    requestedTorque, wheelRate, p)
%BALLBOTDCMOTORTORQUEENVELOPE Apply FIT0521 and MDD3A limits.

requestedTorque = requestedTorque(:);
speedScale = max(0, 1 - abs(wheelRate)/p.motor.noLoadSpeed);
availableTorque = min(p.driver.continuousTorqueLimit, ...
    p.motor.stallTorque*speedScale);
limitedTorque = min(max(requestedTorque, -availableTorque), ...
    availableTorque);
end
