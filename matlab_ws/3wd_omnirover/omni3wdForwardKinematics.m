function bodyTwist = omni3wdForwardKinematics(wheelSpeed, ...
    wheelAngles, wheelRadius, wheelCenterRadius)
%OMNI3WDFORWARDKINEMATICS Convert wheel speeds to a body-frame twist.

arguments
    wheelSpeed (3,1) double
    wheelAngles (1,3) double
    wheelRadius (1,1) double {mustBePositive}
    wheelCenterRadius (1,1) double {mustBePositive}
end

kinematicMatrix = [-sin(wheelAngles(:)), cos(wheelAngles(:)), ...
    wheelCenterRadius*ones(3,1)];
bodyTwist = kinematicMatrix \ (wheelRadius*wheelSpeed);
end

