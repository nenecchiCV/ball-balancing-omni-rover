function p = ballbotFullPlantParameters(sampleTime, design)
%BALLBOTFULLPLANTPARAMETERS Parameters for the Full Multibody LQI model.
%   Optional SAMPLETIME [s] overrides the default 5 ms estimator/
%   controller rate (see ballbotParameters).  Optional DESIGN is the
%   pre-derivation override struct forwarded to ballbotParameters.

if nargin < 1
    sampleTime = [];
end
if nargin < 2
    design = [];
end
p = ballbotParameters(sampleTime, design);

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
% Lateral mount offset [m] of the payload rod in the chassis frame
% (x/y).  Zero keeps the rod on-axis; nonzero shifts the assembled
% rover center of mass by payload.mass/rover.mass*offset and is applied
% to PayloadRodMount at runtime by ballbotRev1PlantAugment.  Used to
% test controller robustness against an off-center payload.
p.fullplant.payload.lateralOffset = [0; 0];
p.fullplant.payload.originalRoverMass = p.rover.mass;
% Combine the chassis lump (frame stack, mounts, electronics), the three
% motor/wheel pods at wheel-center height, and the payload rod.
wheelCenterAboveBall = p.wheel.geometry.centerFromBall(3, 1);
baseComAboveBall = (p.rover.bodyMass*p.rover.bodyComAboveBall + ...
    3*(p.motor.mass + p.wheel.mass)*wheelCenterAboveBall)/p.rover.mass;
payloadComAboveBall = p.rover.centerAboveBall + ...
    p.fullplant.payload.centerOffset;
p.rover.mass = p.rover.mass + p.fullplant.payload.mass;
% Keep centerAboveBall as the chassis-frame geometry reference.  The LQI
% reduced plant uses the combined rover/payload center of mass instead.
p.rover.comAboveBall = (p.fullplant.payload.originalRoverMass* ...
    baseComAboveBall + p.fullplant.payload.mass*payloadComAboveBall)/ ...
    p.rover.mass;
p.fullplant.payload.comAboveChassis = ...
    p.rover.comAboveBall - p.rover.centerAboveBall;
% Tilt inertia of the assembled rover about its combined COM (x/y axis).
p.rover.tiltInertiaCom = p.rover.bodyInertia(1) + ...
    p.rover.bodyMass*(p.rover.bodyComAboveBall - p.rover.comAboveBall)^2 + ...
    3*(p.motor.mass*(3*0.0122^2 + 0.052^2)/12 + ...
       p.motor.mass*(wheelCenterAboveBall - p.rover.comAboveBall)^2 + ...
       p.wheel.mass*p.wheel.radius^2/2 + ...
       p.wheel.mass*(wheelCenterAboveBall - p.rover.comAboveBall)^2) + ...
    p.fullplant.payload.mass*p.fullplant.payload.length^2/12 + ...
    p.fullplant.payload.mass*(payloadComAboveBall - p.rover.comAboveBall)^2;

% Maintain all three wheel/ball contacts through the normal-control region
% without a kinematic retention constraint.  A 0.20 mm preload is the
% smallest tested value that retained contact at 17 degrees; larger values
% (0.50 mm and above) produced excessive penalty-contact reactions.
% 0.40 mm retested after the contacts became near-critically damped: the
% deeper engagement doubles the tilt at which a wheel geometrically
% unloads (preload depth ~ tilt*ballRadius*cos(contactLatitude)), which
% is the margin the ball-escape transient needs.
p.wheel.contactPreload = 4.0e-4;
p.wheel.geometry = ballbotWheelGeometry(p);
rollingXY = p.wheel.geometry.rollingBody(1:2, :).';
p.controller.lqi.wheelSpeedFromPlanarVelocity = [ ...
    -(sin(p.wheel.contactLatitude)/p.wheel.radius)*rollingXY, ...
    (p.rover.chassisRadius/p.wheel.radius)*ones(3, 1)];
p.wheel.normalLoadNominal = ...
    p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude));
p.wheel.contactTorqueLimit = p.contact.wheelBall.tractionEstimateFriction* ...
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
% Use the analytic spring sag: the previous 1.43e-4 m constant was an
% empirical zero-search result for the old 3e5 N/m stiffness and does
% not transfer after the contact softening.
p.fullplant.groundStaticDeflection = p.fullplant.linearSpringDeflection;
% The wheels also carry the rover through the wheel-ball penalty
% contacts: with the chassis placed at the geometric (zero-penetration)
% height, it free-drops ~N/k at t=0 and the asymmetric grab is the
% startup tilt kick (~0.01-0.02 rad).  Sink the chassis by the static
% wheel-ball deflection projected on the vertical.
p.fullplant.wheelBallStaticDeflection = p.wheel.normalLoadNominal/ ...
    p.contact.wheelBall.normalStiffness/sin(p.wheel.contactLatitude);
p.rover.initialPositionWorld(3) = p.ball.radius + ...
    p.rover.centerAboveBall - p.fullplant.groundStaticDeflection - ...
    p.fullplant.wheelBallStaticDeflection;
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
% Enter recovery before commanded wheel action unloads the upper contact.
% Lowered from 17 deg to 5 deg so that hardware-realistic tilted starts
% (placement tolerance <10 deg) engage the righting controller instead
% of the balance gains, which cannot arrest a 10 deg topple (they only
% request ~0.2 N*m vs the ~0.44 N*m gravity moment).  Normal-cycle
% transients stay under ~3 deg, keeping recovery out of ordinary moves.
p.controller.recoveryTilt = deg2rad(5.0);

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

% Identifies which builder produced this set so a model InitFcn can
% regenerate ballbotParams when a stale variant is left in the
% workspace.
p.parameterSource = "ballbotFullPlantParameters";
end
