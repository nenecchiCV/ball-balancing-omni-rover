function output = ballbotTorqueDutyStep( ...
    torqueCommand, wheelSpeed, previousIntegral, mode, p)
%BALLBOTTORQUEDUTYSTEP Wheel-torque command to MDD3A duty for rev1 drive.
%   The voltage-domain speed PI (ballbotSpeedPiDutyStep) closes the
%   wheel-rate loop at roughly 6 rad/s, slower than the ~6.4 rad/s
%   unstable tilt pole, so the balance loop cannot recover.  Here the
%   upstream LQI issues wheel torque directly and this step only inverts
%   the DC motor equation to obtain the PWM duty:
%       duty = (R_total*I + Ke*w)/Vs,  I = tau/Kt,
%       R_total = armatureResistance + bridgeOnResistance.
%   Torque therefore responds at the sub-millisecond electrical time
%   scale, and encoder quantization no longer enters the fast channel.
%
%   When p.controller.torqueDrive.enabled is false the first input is
%   interpreted as a wheel-speed command and the voltage PI is used, so
%   the block works in both configurations.

if ~isfield(p.controller, "torqueDrive") || ...
        ~isfield(p.controller.torqueDrive, "enabled") || ...
        ~p.controller.torqueDrive.enabled || ...
        (isfield(p.controller, "fullplant") && ...
         isfield(p.controller.fullplant, "enabled") && ...
         p.controller.fullplant.enabled) || ...
        mode == 0
    output = ballbotSpeedPiDutyStep(torqueCommand, wheelSpeed, ...
        previousIntegral, mode, p);
    return
end

torqueLimit = min(p.driver.continuousTorqueLimit, ...
    p.controller.tractionLimitedWheelTorque);
torque = min(max(torqueCommand, -torqueLimit), torqueLimit);

% The encoder-quantized wheel rate flickers by quantization/sampleTime
% (~3.7 rad/s at 200 Hz), which would appear in the duty through the
% back-EMF term as +/-0.16 duty noise.  Compensate the EMF with a
% low-passed wheel speed instead; the unused PI memory slot stores the
% filtered value.
if isfield(p.controller.torqueDrive, "backEmfSpeedTimeConstant")
    timeConstant = p.controller.torqueDrive.backEmfSpeedTimeConstant;
else
    timeConstant = 5*p.controller.sampleTime;
end
blend = p.controller.sampleTime/(timeConstant + p.controller.sampleTime);
filteredSpeed = previousIntegral(:) + ...
    blend*(wheelSpeed(:) - previousIntegral(:));

current = torque/p.motor.torqueConstant;
resistance = p.motor.armatureResistance;
if isfield(p.driver, "bridgeOnResistance")
    resistance = resistance + p.driver.bridgeOnResistance;
end
duty = (resistance*current + p.motor.backEmfConstant*filteredSpeed)/ ...
    p.driver.supplyVoltage;
duty = min(max(duty, -p.driver.maxDutyCycle), p.driver.maxDutyCycle);

output = [duty; filteredSpeed];
end
