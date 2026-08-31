function [wheelTorque, achievedBallTorque, saturation] = ...
    ballbotTorqueAllocator(commandedBallTorque, p)
%BALLBOTTORQUEALLOCATOR Map ball torque to the three wheel-shaft torques.

allocation = p.wheel.geometry.ballTorqueFromWheelTorque;
regularization = 1.0e-10;
wheelTorqueRaw = allocation'/(allocation*allocation' + ...
    regularization*eye(3))*commandedBallTorque(:);

limit = p.wheel.commandTorqueLimit;
wheelTorque = min(max(wheelTorqueRaw, -limit), limit);
achievedBallTorque = allocation*wheelTorque;
saturation = wheelTorqueRaw - wheelTorque;
end
