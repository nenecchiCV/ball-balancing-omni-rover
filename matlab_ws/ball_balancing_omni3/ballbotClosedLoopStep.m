function [output, diagnostics] = ballbotClosedLoopStep(input, p)
%BALLBOTCLOSEDLOOPSTEP Stateful wrapper used by the Simulink controller.
% Input: [imu(6); wheelDisplacement(3); command(3); enable].
% Output: [wheelTorque(3); estimate(14); mode].
% Diagnostics: [gyroBiasZ; biasLearningEnabled; qualificationTime].

persistent estimatorState velocityIntegral previousWheelDisplacement ...
    yawBiasReady

if isempty(estimatorState)
    estimatorState = ballbotEstimatorInitialState;
    velocityIntegral = zeros(2, 1);
    previousWheelDisplacement = input(7:9);
    yawBiasReady = false;
end

imu = input(1:6);
wheelDisplacement = input(7:9);
command = input(10:12);
enable = input(13) ~= 0;
wheelRate = ballbotWheelRateFromDisplacement(wheelDisplacement, ...
    previousWheelDisplacement, p.estimator.sampleTime);
previousWheelDisplacement = wheelDisplacement;

[estimatorState, estimate, estimatorDiagnostics] = ballbotEstimatorStep( ...
    estimatorState, imu, wheelRate, p);
yawBiasReady = ballbotYawBiasStartupGuard(yawBiasReady, estimate(6), ...
    estimatorDiagnostics(9) ~= 0, p);
yawControlEnabled = yawBiasReady || ...
    abs(command(3)) > p.controller.yawBiasCommandBypassThreshold;
[wheelTorque, velocityIntegral, mode] = ballbotControlStep( ...
    estimate, command, velocityIntegral, enable, p, yawControlEnabled);
wheelTorque = ballbotServoTorqueEnvelope(wheelTorque, wheelRate, p);

output = [wheelTorque; estimate; double(mode)];
diagnostics = estimatorDiagnostics(8:10);
end
