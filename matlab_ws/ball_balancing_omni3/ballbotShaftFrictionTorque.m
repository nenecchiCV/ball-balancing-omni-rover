function torque = ballbotShaftFrictionTorque(wheelRate, p)
%BALLBOTSHAFTFRICTIONTORQUE Smooth Coulomb friction on the wheel shaft.
%   Returns the friction torque [N*m] an Ideal Torque Source applies
%   across the DriveMotor shaft/case nets.  tanh transition approximates
%   breakaway: the torque magnitude reaches the Coulomb level within a
%   fraction of a rad/s, so the geartrain carries the speed-independent
%   loss the pure-viscous motor model lacks.  wheelRate is the wheel
%   joint's sensed relative velocity (shaft vs housing), so the reaction
%   is correctly placed between wheel and chassis.

scale = 1;
if isfield(p, 'motor') && isfield(p.motor, 'shaftFrictionScale')
    scale = p.motor.shaftFrictionScale;
end
coulomb = scale*p.motor.outputCoulombTorque;
transitionSpeed = 0.05;
if isfield(p.motor, 'shaftFrictionTransitionSpeed')
    transitionSpeed = p.motor.shaftFrictionTransitionSpeed;
end
torque = -coulomb*tanh(wheelRate/max(transitionSpeed, eps));
end
