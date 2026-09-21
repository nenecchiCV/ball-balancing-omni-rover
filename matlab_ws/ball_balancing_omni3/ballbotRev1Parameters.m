function p = ballbotRev1Parameters(payloadMass)
%BALLBOTREV1PARAMETERS Parameters for the rev1 Simscape-actuator model.
%   p = BALLBOTREV1PARAMETERS returns the parameters for
%   ball_balancing_omni3_multibody_lqi_rev1.slx: the full-plant parameter
%   set extended with the electrical actuator quantities that the Simscape
%   motor/driver network requires.
%
%   p = BALLBOTREV1PARAMETERS(payloadMass) replaces the mounted payload
%   mass [kg] and recomputes every payload-dependent plant quantity.  The
%   LQI and PI gains intentionally stay at the nominal design so the sweep
%   measures robustness to an unannounced load change.

p = ballbotFullPlantParameters;

if nargin >= 1 && ~isempty(payloadMass)
    p.fullplant.payload.mass = payloadMass;
    p.rover.mass = p.fullplant.payload.originalRoverMass + payloadMass;
    p.fullplant.payload.comAboveChassis = ...
        payloadMass*p.fullplant.payload.centerOffset/p.rover.mass;
    p.rover.comAboveBall = p.rover.centerAboveBall + ...
        p.fullplant.payload.comAboveChassis;
    p.wheel.normalLoadNominal = ...
        p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude));
    p.wheel.contactTorqueLimit = p.contact.wheelBall.dynamicFriction* ...
        p.wheel.normalLoadNominal*p.wheel.radius;
    p.controller.tractionLimitedWheelTorque = min( ...
        p.driver.continuousTorqueLimit, p.wheel.contactTorqueLimit);
    p.controller.theoreticalTractionAcceleration = sqrt(3)* ...
        p.controller.tractionLimitedWheelTorque/(p.wheel.radius* ...
        (p.rover.mass + p.ball.mass));
    p.controller.theoreticalCommandAcceleration = min([ ...
        p.controller.maxPlanarAcceleration, ...
        p.controller.theoreticalTractionAcceleration, ...
        p.controller.theoreticalLeanAcceleration]);
    supportedMass = p.ball.mass + p.rover.mass;
    p.fullplant.linearSpringDeflection = supportedMass*p.gravity/ ...
        p.contact.ballGround.normalStiffness;
    % Scale the empirically fitted static deflection with the supported
    % mass; the reference value was measured at the nominal payload.
    nominalSupportedMass = p.ball.mass + ...
        p.fullplant.payload.originalRoverMass + 0.50;
    p.fullplant.groundStaticDeflection = 1.43e-4* ...
        supportedMass/nominalSupportedMass;
    p.rover.initialPositionWorld(3) = p.ball.radius + ...
        p.rover.comAboveBall - p.fullplant.groundStaticDeflection;
end

% Armature inductance is not in the FIT0521 datasheet.  A typical 6 V
% micro-gearmotor of this size has an electrical time constant well below
% 1 ms; 1 mH gives tau_e = 0.53 ms against the 30 ms mechanical constant.
p.motor.armatureInductance = 1.0e-3;

% MDD3A logic inputs accept 3.3/5 V logic.  The averaged H-Bridge maps the
% PWM input voltage 0..logicHighVoltage onto a 0..100% duty ratio and
% asserts direction reversal above the reverse threshold.
p.driver.logicHighVoltage = 5.0;
p.driver.enableThresholdVoltage = 2.5;
p.driver.reverseThresholdVoltage = 2.5;
p.driver.brakeThresholdVoltage = 2.5;
% TB67H450 H-bridge: combined high+low side on-resistance.
p.driver.bridgeOnResistance = 0.35;
p.driver.bridgeOffConductance = 1.0e-6;

% Repeated move/stop command cycle used by ballbotRev1CommandProfile.
% The planar command ramps at accelerationLimit; the controller already
% caps the lean-equivalent demand through theoreticalCommandAcceleration,
% but ramping the reference keeps the feedforward step from outrunning
% the physical lean the body can establish.
p.command.cyclesEnabled = false;
p.command.cycleStart = 0.5;
p.command.cycleMoveDuration = 1.5;
p.command.cycleStopDuration = 1.0;
p.command.cycleCount = 3;
p.command.accelerationLimit = 0.5;
end
