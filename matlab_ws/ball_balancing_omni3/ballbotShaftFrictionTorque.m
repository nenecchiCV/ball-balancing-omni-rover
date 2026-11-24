function torque = ballbotShaftFrictionTorque(wheelRate, p, wheelIndex)
%BALLBOTSHAFTFRICTIONTORQUE Smooth Coulomb friction on the wheel shaft.
%   Returns the friction torque [N*m] an Ideal Torque Source applies
%   across the DriveMotor shaft/case nets.  tanh transition approximates
%   breakaway: the torque magnitude reaches the Coulomb level within a
%   fraction of a rad/s, so the geartrain carries the speed-independent
%   loss the pure-viscous motor model lacks.  wheelRate is the wheel
%   joint's sensed relative velocity (shaft vs housing), so the reaction
%   is correctly placed between wheel and chassis.
%
%   wheelIndex (optional, default 1) selects the per-wheel Coulomb
%   level when p.motor.outputCoulombTorquePos/Neg are present; sets
%   without the vectors use the scalar outputCoulombTorque for every
%   wheel and direction.  NOTE: the FR-16 drivetrain deadzone is NOT
%   modelled here -- it is clipped in the duty domain inside
%   ballbotMdd3aGateDrive, which reproduces the measured TUNE_DEADZONE
%   input-output behaviour without exciting a comp-kick limit cycle
%   in this smooth-friction channel (see ballbotRev1Parameters).

if nargin < 3
    wheelIndex = 1;
end
scale = 1;
if isfield(p, 'motor') && isfield(p.motor, 'shaftFrictionScale')
    scale = p.motor.shaftFrictionScale;
end
coulomb = p.motor.outputCoulombTorque;
if wheelRate >= 0
    if isfield(p.motor, 'outputCoulombTorquePos') && ...
            numel(p.motor.outputCoulombTorquePos) >= wheelIndex
        coulomb = p.motor.outputCoulombTorquePos(wheelIndex);
    end
elseif isfield(p.motor, 'outputCoulombTorqueNeg') && ...
        numel(p.motor.outputCoulombTorqueNeg) >= wheelIndex
    coulomb = p.motor.outputCoulombTorqueNeg(wheelIndex);
end
transitionSpeed = 0.05;
if isfield(p.motor, 'shaftFrictionTransitionSpeed')
    transitionSpeed = p.motor.shaftFrictionTransitionSpeed;
end
torque = -scale*coulomb*tanh(wheelRate/max(transitionSpeed, eps));
end
