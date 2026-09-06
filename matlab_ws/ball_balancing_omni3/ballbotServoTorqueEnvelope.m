function limitedTorque = ballbotServoTorqueEnvelope( ...
    requestedTorque, wheelRate, p)
%BALLBOTSERVOTORQUEENVELOPE Compatibility wrapper for renamed actuator.

limitedTorque = ballbotDcMotorTorqueEnvelope( ...
    requestedTorque, wheelRate, p);
end
