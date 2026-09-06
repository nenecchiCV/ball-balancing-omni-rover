function design = ballbotDesignLqi(p, design)
%BALLBOTDESIGNLQI Design the planar balance/velocity LQI controller.
% The reduced model represents one horizontal axis about the upright
% equilibrium. Its input is the corresponding ball torque. The complete
% Simscape Multibody plant remains the nonlinear verification plant.

arguments
    p (1, 1) struct
    design (1, 1) struct
end

mass = p.rover.mass;
height = p.rover.centerAboveBall;
bodyRadius = p.rover.chassisRadius;
effectiveInertia = mass*(height^2 + bodyRadius^2/4);

% x = [v; alpha; alphaDot], where positive alpha tilts in the commanded
% travel direction. Positive generalized torque accelerates both travel
% and tilt in this local model.
A = [0, 0, 0; ...
     0, 0, 1; ...
     0, mass*p.gravity*height/effectiveInertia, 0];
B = [1/(mass*height); 0; 1/effectiveInertia];
C = [1, 0, 0];
D = 0;
plant = ss(A, B, C, D);

Q = blkdiag(design.stateWeight, design.integralWeight);
[gain, riccati, poles] = lqi(plant, Q, design.inputWeight);

design.stateGain = gain(1:3);
design.integralGain = gain(4);
design.closedLoopPoles = poles;
design.riccatiSolution = riccati;
design.reducedA = A;
design.reducedB = B;
design.reducedC = C;
design.effectiveInertia = effectiveInertia;
end
