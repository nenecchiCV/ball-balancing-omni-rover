function limitedTorque = ballbotServoTorqueEnvelope( ...
    requestedTorque, wheelRate, p)
%BALLBOTSERVOTORQUEENVELOPE Apply the 16007 torque-speed envelope.

requestedTorque = requestedTorque(:);
wheelRate = wheelRate(:);
speedScale = max(0, 1 - abs(wheelRate)/p.servo.maxSpeed);
limitedTorque = requestedTorque;
accelerating = requestedTorque.*wheelRate > 0;
limitedTorque(accelerating) = requestedTorque(accelerating).* ...
    speedScale(accelerating);
limitedTorque = min(max(limitedTorque, ...
    -p.wheel.commandTorqueLimit), p.wheel.commandTorqueLimit);
end
