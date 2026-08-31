function output = ballbotControllerUpdate( ...
    estimate, command, previousVelocityIntegral, enable, p)
%BALLBOTCONTROLLERUPDATE Pack the stateless controller outputs for Simulink.

[requestedTorque, nextVelocityIntegral, mode] = ballbotControlStep( ...
    estimate, command, previousVelocityIntegral, enable ~= 0, p);
output = [requestedTorque; nextVelocityIntegral; double(mode)];
end
