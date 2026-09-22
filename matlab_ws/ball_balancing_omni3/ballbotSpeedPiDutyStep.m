function output = ballbotSpeedPiDutyStep( ...
    speedCommand, wheelSpeed, previousIntegral, mode, p)
%BALLBOTSPEEDPIDUTYSTEP Speed PI with duty output for the Simscape rev1 drive.
%   The rev1 model replaces the lumped motor equation with a Simscape
%   H-bridge and DC-motor network.  This function keeps the wheel-speed PI
%   and anti-windup of ballbotMdd3aVoltageController, maps the voltage
%   command to a signed duty ratio, and applies a dynamic duty clamp that
%   reproduces the former motor-torque limit as an equivalent armature-
%   current limit:
%       T = Kt*(duty*Vs - Kv*w)/Ra - lam*w  <=  Tlim
%   =>  duty <= (Ilim*Ra + Kv*w)/Vs  with  Ilim = (Tlim + lam*w)/Kt.

controllerOutput = ballbotMdd3aVoltageController( ...
    speedCommand, wheelSpeed, previousIntegral, mode, p);
voltage = controllerOutput(1:3);
nextIntegral = controllerOutput(4:6);

duty = voltage/p.driver.supplyVoltage;

torqueLimit = min(p.driver.continuousTorqueLimit, ...
    p.controller.tractionLimitedWheelTorque);
if isfield(p.controller, "fullplant") && ...
        isfield(p.controller.fullplant, "enabled") && ...
        p.controller.fullplant.enabled
    torqueLimit = min(torqueLimit, ...
        p.controller.fullplant.motorTorqueLimit);
end

currentHigh = (torqueLimit + p.motor.viscousFriction*wheelSpeed)/ ...
    p.motor.torqueConstant;
currentLow = (-torqueLimit + p.motor.viscousFriction*wheelSpeed)/ ...
    p.motor.torqueConstant;
dutyHigh = (currentHigh*p.motor.armatureResistance + ...
    p.motor.backEmfConstant*wheelSpeed)/p.driver.supplyVoltage;
dutyLow = (currentLow*p.motor.armatureResistance + ...
    p.motor.backEmfConstant*wheelSpeed)/p.driver.supplyVoltage;
duty = min(max(duty, dutyLow), dutyHigh);
duty = min(max(duty, -p.driver.maxDutyCycle), p.driver.maxDutyCycle);

output = [duty; nextIntegral];
end
