function output = ballbotClosedLoopStep(input, p)
%BALLBOTCLOSEDLOOPSTEP Stateful wrapper used by the Simulink controller.
% Input: [imu(6); wheelDisplacement(3); command(3); enable].
% Output: [wheelTorque(3); estimate(14); mode].

persistent estimatorState velocityIntegral previousWheelDisplacement

if isempty(estimatorState)
    estimatorState = ballbotEstimatorInitialState;
    velocityIntegral = zeros(2, 1);
    previousWheelDisplacement = input(7:9);
end

imu = input(1:6);
wheelDisplacement = input(7:9);
command = input(10:12);
enable = input(13) ~= 0;
wheelRate = ballbotWheelRateFromDisplacement(wheelDisplacement, ...
    previousWheelDisplacement, p.estimator.sampleTime);
previousWheelDisplacement = wheelDisplacement;

[estimatorState, estimate] = ballbotEstimatorStep( ...
    estimatorState, imu, wheelRate, p);
[wheelTorque, velocityIntegral, mode] = ballbotControlStep( ...
    estimate, command, velocityIntegral, enable, p);
wheelTorque = ballbotServoTorqueEnvelope(wheelTorque, wheelRate, p);

output = [wheelTorque; estimate; double(mode)];
end
