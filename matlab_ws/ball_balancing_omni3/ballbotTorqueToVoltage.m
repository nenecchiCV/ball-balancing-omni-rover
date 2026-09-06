function voltage = ballbotTorqueToVoltage( ...
    torqueCommand, wheelSpeed, mode, p)
%BALLBOTTORQUETOVOLTAGE Estimate motor voltage from shaft torque and speed.
% The steady-state output-shaft motor model is inverted directly. No
% measured or estimated current is used as a controller input or state.

arguments
    torqueCommand (3, 1) double
    wheelSpeed (3, 1) double
    mode (1, 1) double
    p (1, 1) struct
end

if mode == 0
    voltage = zeros(3, 1);
    return
end

torqueCommand = min(max(torqueCommand, ...
    -p.wheel.commandTorqueLimit), p.wheel.commandTorqueLimit);

% tau_shaft = Kt*i - b*omega and V = R*i + Ke*omega. Eliminating i gives
% the feedforward voltage below without using current feedback.
voltageRaw = (p.motor.armatureResistance/p.motor.torqueConstant)* ...
    (torqueCommand + p.motor.viscousFriction*wheelSpeed) + ...
    p.motor.backEmfConstant*wheelSpeed;

voltageLimit = p.driver.maxDutyCycle*p.driver.supplyVoltage;
voltage = min(max(voltageRaw, -voltageLimit), voltageLimit);
end
