function output = ballbotMdd3aGateDrive(duty, p, wheelIndex)
%BALLBOTMDD3AGATEDRIVE Map a signed duty command to MDD3A gate voltages.
%   Returns the PWM-port and REV-port voltages that drive the averaged
%   H-Bridge: |duty| scales the PWM input (0..logicHigh maps to 0..100%%),
%   and REV is asserted when the command is negative.  BRK is not used;
%   the controller brakes by commanding the opposite rotation direction.
%
%   wheelIndex (optional, default 1) selects the per-wheel drivetrain
%   deadzone: below p.drive.deadzonePlantPos/Neg(wheelIndex) the
%   gearmotor stall torque cannot break the stiction the firmware
%   TUNE_DEADZONE sweep measures, so no effective voltage reaches the
%   winding.  The rev2 wheel servo lifts nonzero commands by
%   p.drive.deadzoneCompPos/Neg (wheel_servo.cpp parity); when the
%   plant vectors equal the comp values the clip restores exactly the
%   commanded duty -- the same cancellation the firmware assumes.
%   Parameter sets without the plant fields apply no deadzone.

if nargin < 3
    wheelIndex = 1;
end
duty = min(max(duty, -p.driver.maxDutyCycle), p.driver.maxDutyCycle);
deadzone = 0;
if isfield(p, 'drive') && isfield(p.drive, 'deadzonePlantPos') && ...
        numel(p.drive.deadzonePlantPos) >= wheelIndex
    if duty > 0
        deadzone = p.drive.deadzonePlantPos(wheelIndex);
    elseif duty < 0 && isfield(p.drive, 'deadzonePlantNeg') && ...
            numel(p.drive.deadzonePlantNeg) >= wheelIndex
        deadzone = p.drive.deadzonePlantNeg(wheelIndex);
    end
end
effectiveDuty = sign(duty)*max(0, abs(duty) - deadzone);
pwmVoltage = abs(effectiveDuty)*p.driver.logicHighVoltage;
reverseVoltage = p.driver.logicHighVoltage*double(effectiveDuty < 0);
output = [pwmVoltage; reverseVoltage];
end
