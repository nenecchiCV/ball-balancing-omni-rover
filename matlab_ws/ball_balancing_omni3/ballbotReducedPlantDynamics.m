function stateDerivative = ballbotReducedPlantDynamics(state, wheelCommand, p)
%BALLBOTREDUCEDPLANTDYNAMICS Constrained low-frequency upright plant.

velocity = state(1:2);
tilt = state(3:4);
tiltRate = state(5:6);
yawRate = state(7);
wheelRate = state(8:10);
geometry = p.controller.lqi.wheelSpeedFromPlanarVelocity;
planarVelocity = pinv(geometry)*wheelRate;
innerBandwidth = 1/p.motor.timeConstant;
driveAcceleration = innerBandwidth*(planarVelocity(1:2) - velocity);
yawAcceleration = innerBandwidth*(planarVelocity(3) - yawRate);
mass = p.rover.mass;
height = p.rover.centerAboveBall;
bodyRadius = p.rover.chassisRadius;
effectiveInertia = mass*(height^2 + bodyRadius^2/4);
gravityGain = mass*p.gravity*height/effectiveInertia;
accelerationGain = mass*height/effectiveInertia;
stateDerivative = [driveAcceleration; tiltRate; ...
    gravityGain*tilt - accelerationGain*driveAcceleration; ...
    yawAcceleration; innerBandwidth*(wheelCommand - wheelRate)];
end
