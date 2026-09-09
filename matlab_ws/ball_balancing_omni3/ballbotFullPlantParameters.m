function p = ballbotFullPlantParameters
%BALLBOTFULLPLANTPARAMETERS Parameters for the Full Multibody LQI model.

p = ballbotParameters;

% The nominal speed PI is too aggressive for compliant wheel/ball contact.
p.fullplant.speedPiGainScale = 0.25;
p.motor.speedControllerKp = p.fullplant.speedPiGainScale* ...
    p.motor.speedControllerKp;
p.motor.speedControllerKi = p.fullplant.speedPiGainScale* ...
    p.motor.speedControllerKi;
p.motor.speedControllerAntiWindup = 1/max(p.motor.speedControllerKp, eps);
p.motor.speedControllerIntegralLimit = ...
    p.driver.supplyVoltage/p.motor.speedControllerKi;

% High-mounted ballast used to raise the rover center of mass.  The model
% represents the rod as a slender 20 mm square, 300 mm long rigid solid.
p.fullplant.payload.mass = 0.50;
p.fullplant.payload.length = 0.30;
p.fullplant.payload.width = 0.020;
p.fullplant.payload.centerOffset = p.rover.chassisHeight/2 + ...
    p.fullplant.payload.length/2;
p.fullplant.payload.originalRoverMass = p.rover.mass;
p.rover.mass = p.rover.mass + p.fullplant.payload.mass;
p.fullplant.payload.comAboveChassis = ...
    p.fullplant.payload.mass*p.fullplant.payload.centerOffset/p.rover.mass;
% Keep centerAboveBall as the chassis-frame geometry reference.  The LQI
% reduced plant uses the combined rover/payload center of mass instead.
p.rover.comAboveBall = p.rover.centerAboveBall + ...
    p.fullplant.payload.comAboveChassis;

% Maintain all three wheel/ball contacts through the normal-control region
% without a kinematic retention constraint.  A 0.20 mm preload is the
% smallest tested value that retained contact at 17 degrees; larger values
% (0.50 mm and above) produced excessive penalty-contact reactions.
p.wheel.contactPreload = 2.0e-4;
p.wheel.geometry = ballbotWheelGeometry(p);
rollingXY = p.wheel.geometry.rollingBody(1:2, :).';
p.controller.lqi.wheelSpeedFromPlanarVelocity = [ ...
    -(sin(p.wheel.contactLatitude)/p.wheel.radius)*rollingXY, ...
    (p.rover.chassisRadius/p.wheel.radius)*ones(3, 1)];
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

% Start the penalty contact at its static-load deflection.  This avoids
% linearizing at the zero-force edge of the ball-ground contact law.
supportedMass = p.ball.mass + p.rover.mass;
p.fullplant.linearSpringDeflection = supportedMass*p.gravity/ ...
    p.contact.ballGround.normalStiffness;
% The smooth transition region reduces the effective stiffness near first
% contact.  A vertical-acceleration zero search of the actual Multibody
% model gives this equilibrium penetration for the nominal parameters.
p.fullplant.groundStaticDeflection = 1.43e-4;
p.rover.initialPositionWorld(3) = p.ball.radius + ...
    p.rover.centerAboveBall - p.fullplant.groundStaticDeflection;
p.command.velocityWorld = zeros(2, 1);
p.command.yawRate = 0;
p.command.profileEnabled = false;
p.command.profileStart = 0.5;
p.command.profileStop = 2.5;
p.command.profileVelocityWorld = [0.02; 0];
p.controller.fullplant.enabled = true;
p.controller.fullplant.velocityKp = [2.3; 2.3];
p.controller.fullplant.velocityKi = [0; 0];
p.controller.fullplant.tiltKp = [16; 16];
p.controller.fullplant.tiltKd = [1.6; 1.6];
p.controller.fullplant.stabilizationSign = 1;
p.controller.fullplant.openLoopEnabled = false;
p.controller.fullplant.motorTorqueLimit = 0.010;
p.controller.fullplant.commandScale = 0.60;
p.controller.fullplant.commandFeedforward = 1.0;
p.controller.minimumContactConfidence = 0;
% Enter recovery before commanded wheel action unloads the upper contact.
% With the unconstrained plant, all contacts remained active through 17.0
% degrees, while loss first appeared at 17.25 degrees in the tested plane.
p.controller.recoveryTilt = deg2rad(17.0);

% Redesign the reduced-order LQI for the formal high-center-of-mass
% hardware configuration.  ballbotParameters computes the baseline gains
% before the payload properties are known, so they must be recomputed here.
p.controller.lqi = ballbotDesignLqi(p, p.controller.lqi);

designFile = fullfile(fileparts(mfilename("fullpath")), ...
    "fullplant_lqi_design.mat");
if isfile(designFile)
    data = load(designFile, "fullplantDesign");
    p.controller.lqi.stateGain = data.fullplantDesign.stateGain;
    p.controller.lqi.integralGain = data.fullplantDesign.integralGain;
    p.controller.lqi.trackingGainScale = ...
        data.fullplantDesign.trackingGainScale;
end
end
