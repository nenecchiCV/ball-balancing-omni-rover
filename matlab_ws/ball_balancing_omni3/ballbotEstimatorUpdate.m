function output = ballbotEstimatorUpdate(previousState, imu, wheelRate, p)
%BALLBOTESTIMATORUPDATE Pack the stateless estimator outputs for Simulink.

[nextState, estimate] = ballbotEstimatorStep( ...
    previousState, imu, wheelRate, p);
output = [nextState; estimate];
end
