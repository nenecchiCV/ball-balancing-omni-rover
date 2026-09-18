function output = ballbotMdd3aGateDrive(duty, p)
%BALLBOTMDD3AGATEDRIVE Map a signed duty command to MDD3A gate voltages.
%   Returns the PWM-port and REV-port voltages that drive the averaged
%   H-Bridge: |duty| scales the PWM input (0..logicHigh maps to 0..100%%),
%   and REV is asserted when the command is negative.  BRK is not used;
%   the controller brakes by commanding the opposite rotation direction.

duty = min(max(duty, -p.driver.maxDutyCycle), p.driver.maxDutyCycle);
pwmVoltage = abs(duty)*p.driver.logicHighVoltage;
reverseVoltage = p.driver.logicHighVoltage*double(duty < 0);
output = [pwmVoltage; reverseVoltage];
end
