function design = ballbotDesignLqi(p, design)
%BALLBOTDESIGNLQI Linearize and design the 10-state three-wheel MIMO LQI.

arguments
    p (1, 1) struct
    design (1, 1) struct
end

stateCount = 10;
inputCount = 3;
x0 = zeros(stateCount, 1);
u0 = zeros(inputCount, 1);
stateStep = design.linearizationStateStep(:);
inputStep = design.linearizationInputStep(:);
A = zeros(stateCount);
for column = 1:stateCount
    delta = zeros(stateCount, 1);
    delta(column) = stateStep(column);
    A(:, column) = (ballbotReducedPlantDynamics(x0 + delta, u0, p) - ...
        ballbotReducedPlantDynamics(x0 - delta, u0, p))/(2*delta(column));
end
B = zeros(stateCount, inputCount);
for column = 1:inputCount
    delta = zeros(inputCount, 1);
    delta(column) = inputStep(column);
    B(:, column) = (ballbotReducedPlantDynamics(x0, u0 + delta, p) - ...
        ballbotReducedPlantDynamics(x0, u0 - delta, p))/(2*delta(column));
end
Cvelocity = [eye(2), zeros(2, stateCount - 2)];
augmentedA = [A, zeros(stateCount, 2); -Cvelocity, zeros(2)];
augmentedB = [B; zeros(2, inputCount)];
Q = diag(1./[design.maximumState(:); ...
    design.maximumVelocityIntegral(:)].^2);
R = diag(1./design.maximumWheelSpeedCommand(:).^2);
[gain, riccati, poles] = lqr(augmentedA, augmentedB, Q, R);
design.stateGain = gain(:, 1:stateCount);
design.integralGain = gain(:, stateCount + (1:2));
design.closedLoopPoles = poles;
design.riccatiSolution = riccati;
design.linearizedA = A;
design.linearizedB = B;
design.velocityOutput = Cvelocity;
design.operatingState = x0;
design.operatingInput = u0;
design.controllabilityRank = rank(ctrb(A, B));
unstablePoles = eig(augmentedA);
unstablePoles = unstablePoles(real(unstablePoles) >= -1.0e-9);
design.augmentedStabilizable = all(arrayfun(@(pole) ...
    rank([pole*eye(size(augmentedA)) - augmentedA, augmentedB]) == ...
    size(augmentedA, 1), unstablePoles));
end
