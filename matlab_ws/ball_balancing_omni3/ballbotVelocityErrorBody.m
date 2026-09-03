function velocityErrorBody = ballbotVelocityErrorBody(estimate, command)
%BALLBOTVELOCITYERRORBODY Rotate world-frame velocity error into body axes.

yaw = estimate(3);
rotationBodyFromWorld = [cos(yaw), sin(yaw); -sin(yaw), cos(yaw)];
velocityErrorBody = rotationBodyFromWorld*(command(1:2) - estimate(7:8));
end
