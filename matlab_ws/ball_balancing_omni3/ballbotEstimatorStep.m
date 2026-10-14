function [nextState, estimate, diagnostics] = ballbotEstimatorStep( ...
    previousState, imu, wheelRate, p)
%BALLBOTESTIMATORSTEP IMU/derived-wheel-rate observer update.
% State: [q_WB(4); v_WB(3); omegaBall_W(3); roverMinusBall_B_xy(2);
%         gyroBiasZ; qualificationTime].
% Estimate: [roll; pitch; yaw; p; q; r; vx_W; vy_W; omegaBall_W(3);
%            roverMinusBall_B_xy(2); contactConfidence].
% Diagnostics: [acceleration_W(3); contactResidual(3); residualRms;
%               updatedGyroBiasZ; biasLearningEnabled;
%               updatedQualificationTime].

dt = p.estimator.sampleTime;
quaternion = previousState(1:4);
velocityWorld = previousState(5:7);
ballRateWorldPrevious = previousState(8:10);
relativePositionBodyPrevious = previousState(11:12);
gyroBiasZPrevious = previousState(13);
qualificationTimePrevious = previousState(14);

specificForceBody = imu(1:3);
gyroBodyRaw = imu(4:6);
gyroBody = gyroBodyRaw - [0; 0; gyroBiasZPrevious];
rotationWorldFromBody = quaternionToRotation(quaternion);

accelerationNorm = norm(specificForceBody);
% Attitude correction validity: specific force points along true up only
% when the body is not accelerating.  The norm test alone cannot see
% horizontal acceleration (|f| changes at second order in a), and the
% horizontal gate cannot be trusted either: it is computed from the
% ESTIMATED attitude, and once the attitude error approaches a/g the
% horizontal component of g+a disappears inside the estimate, so the
% gate locks open during motion and the correction starts tracking the
% specific-force direction instead of gravity -- measured runaway: est
% tilt swung to +/-1.8 rad while the true tilt stayed below 0.02 rad
% (cycle_result_g05).  The only acceleration-free witness independent
% of the estimated attitude is the motion sensors themselves: wheels
% at rest and gyro quiescent means the chassis is genuinely parked, so
% gravity really is the only specific force.  During motion the
% attitude integrates gyro only; the move/stop cycle re-corrects at
% every stop, bounding the drift a noisy IMU accumulates per leg.
quasiStatic = max(abs(wheelRate)) <= p.estimator.biasWheelRateThreshold && ...
    hypot(gyroBody(1), gyroBody(2)) <= p.estimator.biasRollPitchRateThreshold && ...
    abs(gyroBody(3)) <= p.estimator.biasYawRateThreshold;
if abs(accelerationNorm - p.gravity) <= p.estimator.accelNormGate && ...
        quasiStatic
    measuredUpBody = specificForceBody/max(accelerationNorm, eps);
    predictedUpBody = rotationWorldFromBody'*[0; 0; 1];
    correction = p.estimator.attitudeCorrectionGain* ...
        cross(measuredUpBody, predictedUpBody);
    % Clamp the correction rate: even a legitimate gate opening must not
    % snap the attitude, because a large single-step rotation outruns
    % the controller's tilt response.  ~0.5 rad/s still reconverges a
    % 0.05 rad offset in ~0.1 s of real standstill.
    maxCorrectionRate = 0.5;
    correctionMagnitude = norm(correction);
    if correctionMagnitude > maxCorrectionRate
        correction = correction*(maxCorrectionRate/correctionMagnitude);
    end
else
    correction = zeros(3, 1);
end

correctedRateBody = gyroBody + correction;
quaternion = integrateQuaternion(quaternion, correctedRateBody, dt);
rotationWorldFromBody = quaternionToRotation(quaternion);

gravityWorld = [0; 0; -p.gravity];
accelerationWorld = rotationWorldFromBody*specificForceBody + gravityWorld;
velocityWorld = p.estimator.velocityLeak*velocityWorld + ...
    dt*accelerationWorld;
velocityWorld(3) = 0;

geometry = p.wheel.geometry;
normalWorld = rotationWorldFromBody*geometry.normalBody;
rollingWorld = rotationWorldFromBody*geometry.rollingBody;
wheelCenterWorld = rotationWorldFromBody*geometry.centerFromBody;
bodyRateWorld = rotationWorldFromBody*gyroBody;

ballRadius = p.ball.radius;
groundRollingMatrix = [0, ballRadius, 0; ...
    -ballRadius, 0, 0; 0, 0, 0];
kinematicMatrix = zeros(3, 3);
kinematicRightSide = zeros(3, 1);
for wheelIndex = 1:3
    surfaceVelocityMatrix = groundRollingMatrix - ...
        ballRadius*skewMatrix(normalWorld(:, wheelIndex));
    kinematicMatrix(wheelIndex, :) = ...
        rollingWorld(:, wheelIndex)'*surfaceVelocityMatrix;
    wheelCenterVelocity = velocityWorld + ...
        cross(bodyRateWorld, wheelCenterWorld(:, wheelIndex));
    kinematicRightSide(wheelIndex) = ...
        rollingWorld(:, wheelIndex)'*wheelCenterVelocity - ...
        p.wheel.radius*wheelRate(wheelIndex);
end

regularization = p.estimator.kinematicRegularization;
ballRateKinematic = (kinematicMatrix'*kinematicMatrix + ...
    regularization*eye(3))\(kinematicMatrix'*kinematicRightSide);
% Stillness gate: when the assembly is at rest a ball on the ground
% cannot sustain rotation, so any nonzero solve at that point is
% quantization noise, not spin.  The RAW rate cannot decide this: a
% wheel rolling at ball speeds up to ~0.13 m/s turns slower than the
% ~3.7 rad/s per-sample flicker bound, so the raw-rate test fired
% during normal slow translation and halved the velocity estimate.
% Decide on a low-passed rate instead: at rest the symmetric flicker
% averages out to ~0, while sustained rolling keeps a coherent rate,
% shrinking the blind band to ~0.03 m/s.
persistent filteredWheelRateForStill
stillnessFilterTau = 0.080;
stillnessRateThreshold = 1.5;
if isfield(p.estimator, "stillnessFilterTimeConstant")
    stillnessFilterTau = p.estimator.stillnessFilterTimeConstant;
end
if isfield(p.estimator, "ballRateStillWheelRateFiltered")
    stillnessRateThreshold = p.estimator.ballRateStillWheelRateFiltered;
end
if isempty(filteredWheelRateForStill)
    filteredWheelRateForStill = wheelRate;
else
    wStill = dt/(stillnessFilterTau + dt);
    filteredWheelRateForStill = filteredWheelRateForStill + ...
        wStill*(wheelRate - filteredWheelRateForStill);
end
assemblyStill = max(abs(filteredWheelRateForStill)) <= ...
    stillnessRateThreshold && ...
    norm(gyroBody) <= p.estimator.ballRateStillGyroRate;
if assemblyStill
    ballRateKinematic(:) = 0;
end
filterWeight = dt/(p.estimator.ballRateTimeConstant + dt);
ballRateWorld = ballRateWorldPrevious + ...
    filterWeight*(ballRateKinematic - ballRateWorldPrevious);

ballVelocityWorld = groundRollingMatrix*ballRateWorld;
% Velocity anchor: the accel-integrated rover velocity drifts because no
% channel corrects it during motion (a small sustained accel bias
% integrates to ~0.3 m/s over a move leg -- observed).  The
% wheel-derived ball velocity is an independent physical measurement,
% and the rover-ball relative speed is bounded to ~0.05 m/s while the
% contacts hold, so pulling velocityWorld toward ballVelocityWorld
% drains the integrator bias without hiding real relative motion.
anchorMinSpeed = 0.05;
if isfield(p.estimator, "velocityAnchorMinSpeed")
    anchorMinSpeed = p.estimator.velocityAnchorMinSpeed;
end
% Stillness drain: wheels at rest roll nothing, so a still assembly has
% zero world velocity -- any nonzero velocityWorld while assemblyStill
% holds is pure integrator bias (observed: -0.54 m/s phantom while the
% rover sat parked).  Decay it fast; the filtered stillness test is
% blind only below ~0.03 m/s, which this drain rate also caps.
if assemblyStill
    drainTau = 0.1;
    if isfield(p.estimator, "stillVelocityDrainTimeConstant")
        drainTau = p.estimator.stillVelocityDrainTimeConstant;
    end
    velocityWorld(1:2) = velocityWorld(1:2)*(drainTau/(drainTau + dt));
elseif isfield(p.estimator, "velocityAnchorTimeConstant") && ...
        p.estimator.velocityAnchorTimeConstant > 0 && ...
        norm(ballVelocityWorld) > anchorMinSpeed
    anchorWeight = dt/(p.estimator.velocityAnchorTimeConstant + dt);
    velocityWorld(1:2) = velocityWorld(1:2) + ...
        anchorWeight*(ballVelocityWorld(1:2) - velocityWorld(1:2));
end
relativeVelocityWorld = velocityWorld - ballVelocityWorld;
yaw = rotationToEuler(rotationWorldFromBody);
rotationWorldFromYaw = [cos(yaw(3)), -sin(yaw(3)); ...
    sin(yaw(3)), cos(yaw(3))];
relativePositionWorldPrevious = rotationWorldFromYaw* ...
    relativePositionBodyPrevious;
relativePositionWorld = p.estimator.relativePositionLeak* ...
    relativePositionWorldPrevious + dt*relativeVelocityWorld(1:2);
% Physical bound: the rover cannot be further from the ball than the
% ball radius plus the contact latitude sag -- a larger estimate is
% always integrator windup, and unbounded values poison every relPos
% consumer.
relativePositionNorm = norm(relativePositionWorld);
if relativePositionNorm > p.estimator.relativePositionLimit
    relativePositionWorld = relativePositionWorld* ...
        (p.estimator.relativePositionLimit/relativePositionNorm);
end
relativePositionBody = rotationWorldFromYaw'*relativePositionWorld;

residual = kinematicMatrix*ballRateWorld - kinematicRightSide;
residualRootMeanSquare = sqrt(mean(residual.^2));
contactConfidence = exp(-(residualRootMeanSquare/ ...
    p.estimator.contactResidualScale)^2);

qualifiedPreviously = qualificationTimePrevious >= ...
    p.estimator.biasQualificationTime;
if qualifiedPreviously
    thresholdScale = p.estimator.biasQualificationHysteresis;
    contactConfidenceThreshold = ...
        p.estimator.biasContactConfidenceRelease;
else
    thresholdScale = 1;
    contactConfidenceThreshold = ...
        p.estimator.biasContactConfidenceThreshold;
end

wheelRateLow = max(abs(wheelRate)) <= thresholdScale* ...
    p.estimator.biasWheelRateThreshold;
accelerationNormNominal = abs(accelerationNorm - p.gravity) <= ...
    thresholdScale*p.estimator.biasAccelNormThreshold;
rollPitchRateLow = hypot(gyroBody(1), gyroBody(2)) <= ...
    thresholdScale*p.estimator.biasRollPitchRateThreshold;
yawRateLow = abs(gyroBody(3)) <= thresholdScale* ...
    p.estimator.biasYawRateThreshold;
contactReliable = contactConfidence >= contactConfidenceThreshold;
qualificationCandidate = wheelRateLow && accelerationNormNominal && ...
    rollPitchRateLow && yawRateLow && contactReliable;

if qualificationCandidate
    qualificationTime = min(qualificationTimePrevious + dt, ...
        p.estimator.biasQualificationTime);
else
    qualificationTime = 0;
end

biasLearningEnabled = qualificationCandidate && qualifiedPreviously;
biasFilterWeight = dt/(p.estimator.biasTimeConstant + dt);
biasUpdate = double(biasLearningEnabled)*biasFilterWeight* ...
    (gyroBodyRaw(3) - gyroBiasZPrevious);
biasUpdateLimit = p.estimator.biasMaximumUpdatePerSample;
biasUpdate = min(max(biasUpdate, -biasUpdateLimit), biasUpdateLimit);
gyroBiasZ = gyroBiasZPrevious + biasUpdate;
gyroBiasZ = min(max(gyroBiasZ, -p.estimator.biasMaximum), ...
    p.estimator.biasMaximum);

nextState = [quaternion; velocityWorld; ballRateWorld; ...
    relativePositionBody; gyroBiasZ; qualificationTime];
estimate = [yaw; gyroBody; velocityWorld(1:2); ballRateWorld; ...
    relativePositionBody; contactConfidence];
diagnostics = [accelerationWorld; residual; residualRootMeanSquare; ...
    gyroBiasZ; double(biasLearningEnabled); qualificationTime];
end

function matrix = skewMatrix(vector)
matrix = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end

function nextQuaternion = integrateQuaternion(quaternion, rate, dt)
omegaMatrix = [0, -rate(1), -rate(2), -rate(3); ...
    rate(1), 0, rate(3), -rate(2); ...
    rate(2), -rate(3), 0, rate(1); ...
    rate(3), rate(2), -rate(1), 0];
nextQuaternion = quaternion + 0.5*dt*omegaMatrix*quaternion;
nextQuaternion = nextQuaternion/max(norm(nextQuaternion), eps);
end

function rotation = quaternionToRotation(quaternion)
quaternion = quaternion/max(norm(quaternion), eps);
w = quaternion(1);
x = quaternion(2);
y = quaternion(3);
z = quaternion(4);
rotation = [1 - 2*(y^2 + z^2), 2*(x*y - z*w), 2*(x*z + y*w); ...
    2*(x*y + z*w), 1 - 2*(x^2 + z^2), 2*(y*z - x*w); ...
    2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
end

function euler = rotationToEuler(rotation)
pitch = asin(min(max(-rotation(3, 1), -1), 1));
roll = atan2(rotation(3, 2), rotation(3, 3));
yaw = atan2(rotation(2, 1), rotation(1, 1));
euler = [roll; pitch; yaw];
end
