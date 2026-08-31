function [wheelSpeed, worldToWheelMatrix] = omni3wdInverseKinematics( ...
    worldCommand, yaw, ...
    wheelAngles, wheelRadius, wheelCenterRadius)
%OMNI3WDINVERSEKINEMATICS Convert a world-frame twist to wheel speeds.
%   WHEELSPEED = OMNI3WDINVERSEKINEMATICS(WORLDCOMMAND, YAW, ...)
%   converts [vx_W; vy_W; yawRate_W] to the three wheel angular speeds.
%   WORLDTOWHEELMATRIX is the combined 3-by-3 conversion matrix.

arguments
    worldCommand (3,1) double
    yaw (1,1) double
    wheelAngles (1,3) double
    wheelRadius (1,1) double {mustBePositive}
    wheelCenterRadius (1,1) double {mustBePositive}
end

worldToBodyMatrix = [cos(yaw), sin(yaw), 0; ...
    -sin(yaw), cos(yaw), 0; ...
    0, 0, 1];

bodyToWheelMatrix = 1/wheelRadius * ...
    [-sin(wheelAngles(:)), cos(wheelAngles(:)), ...
    wheelCenterRadius*ones(3,1)];

worldToWheelMatrix = bodyToWheelMatrix * worldToBodyMatrix;
wheelSpeed = worldToWheelMatrix * worldCommand;
end
