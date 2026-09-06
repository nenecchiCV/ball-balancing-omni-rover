function velocityErrorBody = ballbotVelocityErrorBody(estimate, command)
%BALLBOTVELOCITYERRORBODY Rotate world-frame velocity error into body axes.

yaw = estimate(3);
rotationBodyFromWorld = [cos(yaw), sin(yaw); -sin(yaw), cos(yaw)];
% Positive wheel actuation in the Multibody plant produces ball motion
% opposite to the controller's tangential-axis convention.  Apply that
% fixed polarity while rotating the world-frame tracking error.
velocityErrorBody = -rotationBodyFromWorld* ...
    (command(1:2) - estimate(7:8));
end
