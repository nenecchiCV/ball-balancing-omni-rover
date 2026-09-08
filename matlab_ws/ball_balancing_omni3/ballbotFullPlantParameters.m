function p = ballbotFullPlantParameters
%BALLBOTFULLPLANTPARAMETERS Parameters for the Full Multibody LQI model.

p = ballbotParameters;

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
p.controller.minimumContactConfidence = 0;
% Enter recovery before commanded wheel action unloads the upper contact.
% With the unconstrained plant, all contacts remained active through 17.0
% degrees, while loss first appeared at 17.25 degrees in the tested plane.
p.controller.recoveryTilt = deg2rad(17.0);

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
