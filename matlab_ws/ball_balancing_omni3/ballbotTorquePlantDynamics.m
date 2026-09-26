function stateDerivative = ballbotTorquePlantDynamics(state, wheelTorque, p)
%BALLBOTTORQUEPLANTDYNAMICS Torque-input upright reduced plant.
%   Companion to ballbotReducedPlantDynamics for rev1 direct-torque drive.
%   State is identical: [planar velocity(2); tilt(2); tilt rate(2);
%   yaw rate; wheel rate(3)].  The input is wheel torque [N*m] instead of
%   a wheel-rate command, so the wheel-speed servo lag disappears and the
%   contact force reaches the chassis at the electrical time scale.
%
%   Tilt convention matches the controller feedback vector: tilt(1) is
%   pitch (rotation about body +y), tilt(2) is -roll.

geometry = p.wheel.geometry;
wheelMap = p.controller.lqi.wheelSpeedFromPlanarVelocity;
axle = geometry.axleBody;

massEffective = p.rover.mass + p.ball.mass + ...
    p.ball.inertia(1, 1)/p.ball.radius^2;
if isfield(p.rover, "tiltInertiaCom")
    inertiaPivot = p.rover.tiltInertiaCom + ...
        p.rover.mass*p.rover.comAboveBall^2;
    height = p.rover.comAboveBall;
else
    inertiaPivot = p.rover.mass*(p.rover.centerAboveBall^2 + ...
        p.rover.chassisRadius^2/4);
    height = p.rover.centerAboveBall;
end
inertiaYaw = p.rover.bodyInertia(3);
inertiaWheel = p.motor.equivalentInertia;
% Viscous-style coupling that pulls the wheel rate toward the kinematic
% no-slip rate; 100 rad/s models the compliant wheel-ball contact.
contactCoupling = 100*inertiaWheel;

gravityGain = p.rover.mass*p.gravity*height/inertiaPivot;
accelerationGain = p.rover.mass*height/inertiaPivot;

% Wheel torque reaches the ball as a tangential contact force.  The map
% is the power dual of the kinematic wheel-rate map: F = W.'*tau.
planarForce = wheelMap.'*wheelTorque;

% The motor stator reaction applies -tau_i about wheel axle a_i to the
% chassis.  Horizontal components tilt the body, the vertical component
% adds a yaw moment.
reactionTilt = [-axle(2, :); axle(1, :)]/inertiaPivot;
reactionYaw = -axle(3, :)/inertiaYaw;

% Empirical corrections fitted from open-loop torque steps on the
% Simscape plant: the ball's rotational inertia and contact compliance
% weaken the torque->force coupling to ~45% of the rigid model, the yaw
% coupling is ~1.5% of the axle-reaction estimate, and the wheels drag
% each other through the chassis (~10% cross-coupling) with ~2.4x the
% bare rotor inertia.
forceScale = 1;
tiltScale = 1;
yawScale = 1;
wheelDriveMatrix = eye(3)/inertiaWheel;
if isfield(p, "torqueFit")
    fit = p.torqueFit;
    if isfield(fit, "forceScale"), forceScale = fit.forceScale; end
    if isfield(fit, "tiltScale"), tiltScale = fit.tiltScale; end
    if isfield(fit, "yawScale"), yawScale = fit.yawScale; end
    if isfield(fit, "wheelDriveMatrix")
        wheelDriveMatrix = fit.wheelDriveMatrix;
    end
end
planarAcceleration = forceScale*planarForce(1:2)/massEffective;
kinematicRate = wheelMap*[state(1:2); state(7)];

stateDerivative = [ ...
    planarAcceleration; ...
    state(5:6); ...
    gravityGain*state(3:4) - accelerationGain*planarAcceleration + ...
        tiltScale*reactionTilt*wheelTorque; ...
    yawScale*(planarForce(3)/inertiaYaw + reactionYaw*wheelTorque); ...
    wheelDriveMatrix*(wheelTorque - ...
        contactCoupling*(state(8:10) - kinematicRate))];
end
