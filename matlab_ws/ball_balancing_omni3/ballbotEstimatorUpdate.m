function [output, diagnostics] = ballbotEstimatorUpdate( ...
    previousState, imu, wheelRate, p)
%BALLBOTESTIMATORUPDATE Pack the stateless estimator outputs for Simulink.

[nextState, estimate, diagnostics] = ballbotEstimatorStep( ...
    previousState, imu, wheelRate, p);
output = [nextState; estimate; diagnostics(8:10)];
end
