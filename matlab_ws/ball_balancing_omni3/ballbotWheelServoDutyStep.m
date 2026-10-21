function output = ballbotWheelServoDutyStep( ...
    servoContext, wheelRate, previousIntegral, mode, p)
%BALLBOTWHEELSERVODUTYSTEP Inner wheel servo of the rev2 multi-rate split.
%   Runs at p.controller.servoSampleTime (~1 kHz) — the outer task
%   (ballbotLqiControllerUpdateRev2) emits the attitude torque and the
%   kinematic wheel-rate target at 200 Hz, and this task tracks the
%   rate target with a P servo and inverts the total torque to PWM
%   duty through the DC motor equation.  Contact transients therefore
%   meet a sub-millisecond torque channel without raising the
%   estimator/attitude loop rate (encoder quantization noise stays out
%   of the balance task).
%
%   servoContext(1:3) = attitudeTorque  (ZOH from outer task)
%   servoContext(4:6) = wheelRateCommand (ZOH)
%   servoContext(7)   = settleWitness   (post-recovery settle flag)
%   servoContext(8)   = enable
%   wheelRate         = fast-sensed wheel rate (servoSampleTime diff)
%   previousIntegral  = filtered wheel-rate memory (ZOH slot reused as
%                       the back-EMF compensation filter state)
%   output = [duty(3); filteredSpeed(3)]

attitudeTorque = servoContext(1:3);
wheelRateCommand = servoContext(4:6);
settleWitness = servoContext(7) ~= 0;
enable = servoContext(8) ~= 0;

% Back-EMF compensation uses the low-passed wheel speed.  The wheel
% rate is measured on the 5 ms window (rev1 resolution): at the 1 kHz
% task the per-count flicker would be quantization/sampleTime ~ 18.5
% rad/s, which destabilized the duty path in validation.  The 25 ms
% wall-clock smoothing therefore stays a TIME constant -- not a sample
% count -- clocked on the measurement rate.
if isfield(p.controller.torqueDrive, "backEmfSpeedTimeConstant")
    timeConstant = p.controller.torqueDrive.backEmfSpeedTimeConstant;
else
    timeConstant = 0.025;
end
blend = p.controller.servoSampleTime/ ...
    (timeConstant + p.controller.servoSampleTime);
filteredSpeed = previousIntegral(:) + ...
    blend*(wheelRate(:) - previousIntegral(:));

% Elevated traction authority mirrors the outer task: mode 2 arms it,
% the settleWitness flag carries the post-recovery hold.
elevatedAuthority = (mode == 2) || settleWitness;
torqueLimit = min(p.driver.continuousTorqueLimit, ...
    p.controller.tractionLimitedWheelTorque);
if elevatedAuthority && ...
        isfield(p.controller, "recoveryWheelTorqueLimit")
    torqueLimit = min(p.driver.continuousTorqueLimit, ...
        p.controller.recoveryWheelTorqueLimit);
end

% Propulsion wheel-rate servo (was inside the 200 Hz LQI task in rev1).
% The deadzone keeps encoder flicker from biasing the torque; the
% headroom cap yields to the attitude channels; propulsionMinShare
% still guarantees a minimum budget so a saturated static lean can be
% broken; settle cuts the servo entirely (traction-drive pump).
propTorque = zeros(3, 1);
propulsionGain = p.controller.torqueDrive.propulsionGain;
if mode == 1 && propulsionGain ~= 0
    if isfield(p.controller.torqueDrive, "filteredSlipDeadzone")
        slipDeadzone = p.controller.torqueDrive.filteredSlipDeadzone;
    else
        slipDeadzone = p.controller.torqueDrive.slipDeadzone;
    end
    rateError = wheelRateCommand - filteredSpeed;
    rateActive = sign(rateError).*max(abs(rateError) - ...
        slipDeadzone, 0);
    propTorque = min(max(propulsionGain*rateActive, ...
        -p.controller.torqueDrive.propulsionTorqueLimit), ...
        p.controller.torqueDrive.propulsionTorqueLimit);
    headroom = max(torqueLimit - abs(attitudeTorque), 0);
    if isfield(p.controller.torqueDrive, "propulsionMinShare")
        headroom = max(headroom, ...
            p.controller.torqueDrive.propulsionMinShare*torqueLimit);
    end
    propTorque = sign(propTorque).*min(abs(propTorque), headroom);
    if settleWitness
        propTorque(:) = 0;
    end
    propTorque = servoStartupRamp(propTorque, enable, p);
end

torque = min(max(attitudeTorque + propTorque, -torqueLimit), ...
    torqueLimit);

current = torque/p.motor.torqueConstant;
resistance = p.motor.armatureResistance;
if isfield(p.driver, "bridgeOnResistance")
    resistance = resistance + p.driver.bridgeOnResistance;
end
duty = (resistance*current + ...
    p.motor.backEmfConstant*filteredSpeed)/p.driver.supplyVoltage;
duty = min(max(duty, -p.driver.maxDutyCycle), p.driver.maxDutyCycle);

output = [duty; filteredSpeed];
end

function scaled = servoStartupRamp(value, enable, p)
%SERVOSTARTUPRAMP Inner-rate soft start for the propulsion channel.
%   Same purpose as the outer startupRamp but clocked on
%   servoSampleTime so the ramp keeps its wall-clock duration.
if ~isfield(p.controller.torqueDrive, "startupTorqueRampTime") || ...
        p.controller.torqueDrive.startupTorqueRampTime <= 0
    scaled = value;
    return
end
persistent rampClock
if isempty(rampClock) || ~enable
    rampClock = 0;
else
    rampClock = rampClock + p.controller.servoSampleTime;
end
scaled = value*min(1, rampClock/ ...
    p.controller.torqueDrive.startupTorqueRampTime);
end
