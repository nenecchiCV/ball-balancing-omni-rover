function output = ballbotServoVoltageController( ...
    speedCommand, wheelSpeed, previousIntegral, mode, p)
%BALLBOTSERVOVOLTAGECONTROLLER Three independent wheel-speed PI loops.

if mode == 0
    voltage = zeros(3, 1);
    nextIntegral = zeros(3, 1);
else
    error = speedCommand - wheelSpeed;
    rawVoltage = p.servo.speedControllerKp*error + ...
        p.servo.speedControllerKi*previousIntegral;
    voltage = min(max(rawVoltage, -p.servo.nominalVoltage), ...
        p.servo.nominalVoltage);
    nextIntegral = previousIntegral + p.controller.sampleTime*(error + ...
        p.servo.speedControllerAntiWindup*(voltage - rawVoltage));
    limit = p.servo.speedControllerIntegralLimit;
    nextIntegral = min(max(nextIntegral, -limit), limit);
end

output = [voltage; nextIntegral];
end
