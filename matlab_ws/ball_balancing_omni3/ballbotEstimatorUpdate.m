function [output, diagnostics] = ballbotEstimatorUpdate( ...
    previousState, imu, wheelRate, p)
%BALLBOTESTIMATORUPDATE Pack the stateless estimator outputs for Simulink.

[nextState, estimate, diagnostics] = ballbotEstimatorStep( ...
    previousState, imu, wheelRate, p);
% The stored output port is 31 wide: element 28 is an unused pad at
% the estimate tail and 29:31 carry the bias diagnostics that the
% stored SelectYawBiasDiagnostics slice feeds to the controller.
output = [nextState; estimate; 0; diagnostics(4:6)];
diagnostics = [diagnostics; zeros(4, 1)];
end
