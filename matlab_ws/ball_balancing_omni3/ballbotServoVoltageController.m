function output = ballbotServoVoltageController( ...
    speedCommand, wheelSpeed, previousIntegral, mode, p)
%BALLBOTSERVOVOLTAGECONTROLLER Compatibility wrapper for MDD3A control.

output = ballbotMdd3aVoltageController( ...
    speedCommand, wheelSpeed, previousIntegral, mode, p);
end
