function output = ballbotMdd3aVoltageController( ...
    speedCommand, wheelSpeed, previousIntegral, mode, p)
%BALLBOTMDD3AVOLTAGECONTROLLER Three PWM-equivalent speed PI loops.

if mode == 0
    voltage = zeros(3, 1);
    nextIntegral = zeros(3, 1);
else
    error = speedCommand - wheelSpeed;
    rawVoltage = p.motor.speedControllerKp*error + ...
        p.motor.speedControllerKi*previousIntegral;
    voltageLimit = p.driver.maxDutyCycle*p.driver.supplyVoltage;
    voltage = min(max(rawVoltage, -voltageLimit), voltageLimit);
    nextIntegral = previousIntegral + p.controller.sampleTime*(error + ...
        p.motor.speedControllerAntiWindup*(voltage - rawVoltage));
    limit = p.motor.speedControllerIntegralLimit;
    nextIntegral = min(max(nextIntegral, -limit), limit);
end

output = [voltage; nextIntegral];
end
