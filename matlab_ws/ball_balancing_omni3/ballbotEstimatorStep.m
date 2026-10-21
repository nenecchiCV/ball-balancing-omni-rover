function [nextState, estimate, diagnostics] = ballbotEstimatorStep( ...
    previousState, imu, wheelRate, p)
%BALLBOTESTIMATORSTEP IMU/derived-wheel-rate observer update.
% State: [q_WB(4); v_WB(3); omegaBall_W(3); roverMinusBall_B_xy(2);
%         gyroBiasZ; qualificationTime].
% Estimate: [roll; pitch; yaw; p; q; r; vx_W; vy_W; omegaBall_W(3);
%            roverMinusBall_B_xy(2)].
% Diagnostics: [acceleration_W(3); updatedGyroBiasZ; biasLearningEnabled;
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
% Error-state EKF internals: 5-state covariance [delta-theta(3);
% delta-gyroBiasXY(2)] plus the filtered motion-model acceleration and
% the previous kinematic ball velocity it is differentiated from.
% Roll/pitch gyro bias is observable through the accelerometer and is
% estimated online; the YAW gyro bias stays on the qualified-stillness
% learner below (no heading reference makes it unobservable here).
persistent ekfCovariance ekfBiasXY ekfAccelWorld prevBallVelocityWorld
ekfEnabled = isfield(p.estimator, "ekf") && ...
    isfield(p.estimator.ekf, "enabled") && p.estimator.ekf.enabled;
if ekfEnabled && isempty(ekfCovariance)
    ekfBiasXY = zeros(2, 1);
    ekfAccelWorld = zeros(3, 1);
    prevBallVelocityWorld = zeros(3, 1);
    ekfCovariance = diag([ ...
        repmat(p.estimator.ekf.initTiltStd^2, 1, 3), ...
        repmat(p.estimator.ekf.initBiasStd^2, 1, 2)]);
end
if ekfEnabled
    gyroBody = gyroBodyRaw - [ekfBiasXY*p.estimator.ekf.biasEnabled; ...
        gyroBiasZPrevious];
else
    gyroBody = gyroBodyRaw - [0; 0; gyroBiasZPrevious];
end

accelerationNorm = norm(specificForceBody);
% Stillness witness: wheels at rest and gyro quiescent means the
% chassis is genuinely parked, so gravity really is the only specific
% force.  Shared by the legacy gated correction, the EKF measurement
% noise selection, and the bias learner below.
quasiStatic = max(abs(wheelRate)) <= p.estimator.biasWheelRateThreshold && ...
    hypot(gyroBody(1), gyroBody(2)) <= p.estimator.biasRollPitchRateThreshold && ...
    abs(gyroBody(3)) <= p.estimator.biasYawRateThreshold;
if ekfEnabled
    % EKF predict: gyro integration + covariance propagation.
    % Error dynamics dTheta' = -[w]x dTheta - dB, dB' = 0.
    quaternion = integrateQuaternion(quaternion, gyroBody, dt);
    rotationWorldFromBody = quaternionToRotation(quaternion);
    stateTransition = eye(5);
    stateTransition(1:3, 1:3) = eye(3) - dt*skewMatrix(gyroBody);
    stateTransition(1:3, 4:5) = -dt*[eye(2); zeros(1, 2)];
    processNoise = diag([ ...
        repmat(p.estimator.ekf.gyroNoiseDensity^2*dt, 1, 3), ...
        repmat(p.estimator.ekf.biasRandomWalk^2*dt, 1, 2)]);
    ekfCovariance = stateTransition*ekfCovariance* ...
        stateTransition' + processNoise;
else
    rotationWorldFromBody = quaternionToRotation(quaternion);
    % Legacy gated correction: specific force points along true up
    % only when the body is not accelerating, and no gate computable
    % from the estimated attitude can be trusted during motion
    % (measured runaway: est tilt swung to +/-1.8 rad while the true
    % tilt stayed below 0.02 rad, cycle_result_g05).  Correct only
    % under the independent stillness witness.
    if abs(accelerationNorm - p.gravity) <= p.estimator.accelNormGate && ...
            quasiStatic
        measuredUpBody = specificForceBody/max(accelerationNorm, eps);
        predictedUpBody = rotationWorldFromBody'*[0; 0; 1];
        correction = p.estimator.attitudeCorrectionGain* ...
            cross(measuredUpBody, predictedUpBody);
        % Clamp the correction rate: even a legitimate gate opening
        % must not snap the attitude, because a large single-step
        % rotation outruns the controller's tilt response.  ~0.5 rad/s
        % still reconverges a 0.05 rad offset in ~0.1 s of standstill.
        maxCorrectionRate = 0.5;
        correctionMagnitude = norm(correction);
        if correctionMagnitude > maxCorrectionRate
            correction = correction* ...
                (maxCorrectionRate/correctionMagnitude);
        end
    else
        correction = zeros(3, 1);
    end
    correctedRateBody = gyroBody + correction;
    quaternion = integrateQuaternion(quaternion, correctedRateBody, dt);
    rotationWorldFromBody = quaternionToRotation(quaternion);
end

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
assemblyStill = max(abs(filteredWheelRateForStill)) <= ...
    stillnessRateThreshold && ...
    norm(gyroBody) <= p.estimator.ballRateStillGyroRate;
% Hold-time gate shared with the stillness drain below: flag flicker at
% the blind-band edge (~0.025 m/s creep) must NOT zero the rate solve --
% that hides real ball motion from the controller (observed: payload
% build crept at ~0.02-0.04 m/s with estV ~0.005 until the lean walked
% off).  Only sustained stillness proves the solved rate is phantom.
persistent stillnessHold
if assemblyStill
    if isempty(stillnessHold)
        stillnessHold = 0;
    end
    stillnessHold = stillnessHold + dt;
else
    stillnessHold = 0;
end
drainHoldTime = 1.0;
if isfield(p.estimator, "stillVelocityDrainHoldTime")
    drainHoldTime = p.estimator.stillVelocityDrainHoldTime;
end
if stillnessHold >= drainHoldTime
    ballRateKinematic(:) = 0;
end
filterWeight = dt/(p.estimator.ballRateTimeConstant + dt);
ballRateWorld = ballRateWorldPrevious + ...
    filterWeight*(ballRateKinematic - ballRateWorldPrevious);

ballVelocityWorld = groundRollingMatrix*ballRateWorld;
if ekfEnabled
    if p.estimator.ekf.useLegacyCorrection
        % A/B experiment: run the proven legacy gated correction inside
        % the EKF predict path (covariance propagation already ran).
        % Gyro integration already happened in the predict step; here
        % the correction is integrated on top, exactly like the legacy
        % branch does.
        if abs(accelerationNorm - p.gravity) <= p.estimator.accelNormGate && ...
                quasiStatic
            measuredUpBody = specificForceBody/max(accelerationNorm, eps);
            predictedUpBody = rotationWorldFromBody'*[0; 0; 1];
            correction = p.estimator.attitudeCorrectionGain* ...
                cross(measuredUpBody, predictedUpBody);
            correctionMagnitude = norm(correction);
            if correctionMagnitude > 0.5
                correction = correction*(0.5/correctionMagnitude);
            end
        else
            correction = zeros(3, 1);
        end
        quaternion = integrateQuaternion(quaternion, correction, dt);
        rotationWorldFromBody = quaternionToRotation(quaternion);
    else
    % Motion-model acceleration: the encoder-witnessed ball velocity is
    % an independent physical channel (not derived from the estimated
    % attitude), so its derivative predicts the rover's world
    % acceleration: rover-ball relative motion is bounded to ~0.05 m/s
    % while the contacts hold, making a_rover ~= a_ball + relPos wobble
    % to first order.  This prediction is what lets the accelerometer
    % correct attitude DURING motion: residual = f - R^T(a_pred - g)
    % isolates gravity regardless of the acceleration state, removing
    % the quasi-static gate the legacy correction needed.  The
    % derivative rides on the already-low-passed ball rate and gets a
    % second filter because encoder flicker differentiates hot.
    % Motion model: chassis translational acceleration predicted from
    % channels independent of the estimated attitude.
    %   a_chassis ~ a_ball + R*d^2(relPos_body)/dt^2
    % a_ball comes from the encoder-witnessed kinematic solve above;
    % relPos (rover-minus-ball horizontal offset) is an estimator
    % state.  The relative term is the piece every simpler model
    % lacked: during balance slosh the light ball micro-rolls ~mm at
    % ~1.5 Hz under the chassis, i.e. relPos_ddot reaches +-0.5-1
    % m/s^2 while the chassis itself barely translates.  a_ball alone
    % read that slosh as chassis accel and steered the estimate half
    % a tilt wrong (measured: est ~true/2, loop self-excited past
    % 0.15 rad); gating on it either leaked ramp-onset accel (ball
    % lags the tip) or deadlocked (witnesses never quiet mid-wobble).
    % relVel needs no differentiation: it is the already-filtered
    % velocityWorld - ballVelocityWorld difference rotated to body.
    accelTau = p.estimator.ekf.accelFilterTimeConstant;
    ballAccelRaw = (ballVelocityWorld - prevBallVelocityWorld)/dt;
    prevBallVelocityWorld = ballVelocityWorld;
    accelWeight = dt/(accelTau + dt);
    ekfAccelWorld = ekfAccelWorld + ...
        accelWeight*(ballAccelRaw - ekfAccelWorld);
    persistent ekfRelVelBody ekfRelAccBody
    if isempty(ekfRelVelBody)
        ekfRelVelBody = zeros(2, 1);
        ekfRelAccBody = zeros(2, 1);
    end
    relVelBodyNow = rotationWorldFromBody(1:2, 1:2)'* ...
        (velocityWorld(1:2) - ballVelocityWorld(1:2));
    relAccRaw = (relVelBodyNow - ekfRelVelBody)/dt;
    ekfRelVelBody = relVelBodyNow;
    relAccWeight = dt/(p.estimator.ekf.relAccFilterTimeConstant + dt);
    ekfRelAccBody = ekfRelAccBody + ...
        relAccWeight*(relAccRaw(1:2) - ekfRelAccBody);
    relAccWorld = rotationWorldFromBody*[ekfRelAccBody; 0];
    % Rotation witness: at ramp onset the chassis tips before it
    % translates, so translation-only predictions (a_ball + relAcc)
    % read ~0 through exactly the window where the IMU feels
    % z_lever*theta_ddot of sway accel.  The gyro-rate derivative
    % reports the tip instantly and is independent of the attitude
    % estimate; its absence let phantom-tilt residuals through at
    % onset (measured: estP pulled negative while true pitch grew).
    persistent ekfGyroDotBody ekfPrevGyroBody
    if isempty(ekfGyroDotBody)
        ekfGyroDotBody = zeros(3, 1);
        ekfPrevGyroBody = gyroBody;
    end
    gyroDotRaw = (gyroBody - ekfPrevGyroBody)/dt;
    ekfPrevGyroBody = gyroBody;
    gyroDotWeight = dt/(p.estimator.ekf.gyroDotFilterTimeConstant + dt);
    ekfGyroDotBody = ekfGyroDotBody + gyroDotWeight*( ...
        gyroDotRaw - ekfGyroDotBody);
    gyroDotWorld = rotationWorldFromBody*ekfGyroDotBody;
    comSwayWorld = p.rover.centerAboveBall* ...
        [gyroDotWorld(2); -gyroDotWorld(1); 0];
    accelPrediction = ekfAccelWorld + relAccWorld + comSwayWorld;
    % The prediction gates update admissibility only -- it is NOT
    % injected into the residual.  Every injection variant (a_ball,
    % a_ball+z*thetaddot, a_ball+relAcc) parasitically cancelled the
    % true tilt signature or fed back through the bootstrapped
    % relPos/velocity channels and destabilized the hold (measured:
    % hold maxTilt 0.15-0.72 vs 0.006 gated-only).  Admissibility =
    % provably inertial segment: wheels at rest, or predicted chassis
    % accel ~0 (steady cruise), where specific force IS pure gravity
    % and the residual is attitude error however large.
    % Rate witness: every accel witness (a_ball, relAcc, z*thetaddot)
    % is filtered and lags the onset by ~0.1-0.2 s; the raw gyro rate
    % has no filter lag -- the instant the chassis tips the gate shuts.
    % Wheel-slew witness: the motors drive the wheels hard the instant
    % a ramp starts, before EITHER accel witness has filtered through
    % (measured: ~0.02 rad of phantom tilt leaked into the estimate
    % during that blind window and stayed, since the gate then shuts
    % legitimately through the whole accel leg).  A lightly filtered
    % wheel slew reacts within ~2 samples.
    persistent ekfWheelSlew ekfPrevWheelRate
    if isempty(ekfWheelSlew)
        ekfWheelSlew = 0;
        ekfPrevWheelRate = wheelRate;
    end
    wheelSlewRaw = max(abs((wheelRate - ekfPrevWheelRate)/dt));
    ekfPrevWheelRate = wheelRate;
    slewWeight = dt/(p.estimator.ekf.wheelSlewFilterTimeConstant + dt);
    ekfWheelSlew = ekfWheelSlew + slewWeight*(wheelSlewRaw - ekfWheelSlew);
    inertialNow = norm(accelPrediction) <= p.estimator.ekf.accelGate && ...
        hypot(gyroBody(1), gyroBody(2)) <= ...
            p.estimator.ekf.gyroRateGate && ...
        ekfWheelSlew <= p.estimator.ekf.wheelSlewGate;
    % Debounce on BOTH inertia witnesses: even instant witnesses admit
    % a ~0.2 s blind window at a ramp onset before the rate builds,
    % and the raw quasiStatic test flickers true inside the calm gaps
    % of an active wobble -- phantom-tilt residuals leaked through
    % exactly those windows (measured: est pulled to ~0.4-0.5x true
    % through every leg, then the tilt ran away; gating without the
    % debounce still fell at 8 s).  Requiring quiet to hold
    % continuously for settleTime before updates resume closes every
    % transient leak; cruise legs and parking stay quiet for seconds,
    % so in-motion corrections still flow -- the EKF gain over the
    % rest-only corrector.
    persistent inertialHold
    if isempty(inertialHold)
        inertialHold = 0;
    end
    if quasiStatic || inertialNow
        inertialHold = inertialHold + dt;
    else
        inertialHold = 0;
    end
    updateAllowed = inertialHold >= p.estimator.ekf.inertialSettleTime;
    if quasiStatic
        modelVar = p.estimator.ekf.modelSigmaStatic^2;
    else
        modelVar = p.estimator.ekf.modelSigmaMove^2;
    end
    expectedForceWorld = -gravityWorld;   % pure gravity (a ~ 0)
    predictedForceBody = rotationWorldFromBody'*expectedForceWorld;
    residual = specificForceBody - predictedForceBody;
    % H = d f / d [dTheta; dB]: with q_true = q ⊗ dq(dTheta) the
    % specific-force sensitivity is [h]x on attitude and zero on bias.
    measurementJacobian = [skewMatrix(predictedForceBody), ...
        zeros(3, 2)];
    accelVar = (p.estimator.ekf.accelNoiseDensity* ...
        sqrt(1/(2*dt)))^2;
    measurementNoise = (accelVar + modelVar)*eye(3);
    innovationCov = measurementJacobian*ekfCovariance* ...
        measurementJacobian' + measurementNoise;
    % While the two-witness gate is open the chassis is provably
    % inertial, so the residual is attitude error however large it
    % is -- vetoing big residuals deadlocked the estimate (measured:
    % once the lag passed ~0.03 rad the 0.3 m/s^2 veto rejected every
    % update and the true tilt walked off to a fall while the estimate
    % sat frozen).  The only remaining veto is the extreme bound that
    % no plausible attitude error reaches: sensor fault, slipping or
    % airborne contact under a still-reading encoder.
    if updateAllowed && ...
            norm(residual) <= p.estimator.ekf.residualGate
        kalmanGain = ekfCovariance*measurementJacobian'/ ...
            innovationCov;
        deltaState = kalmanGain*residual;
        % Per-update tilt clamp: a large residual is legitimate here
        % (the debounced quiet window can open with a real ~0.1 rad
        % backlog), but a single 0.2+ rad snap still outruns the
        % controller's tilt response -- the legacy corrector's
        % 0.5 rad/s rate clamp existed for the same reason.  Bounded
        % steps reconverge a 0.1 rad error in ~4 samples of quiet.
        tiltStepNorm = norm(deltaState(1:3));
        tiltStepLimit = p.estimator.ekf.maxTiltStep;
        if tiltStepNorm > tiltStepLimit
            deltaState(1:3) = deltaState(1:3)* ...
                (tiltStepLimit/tiltStepNorm);
        end
        % Inject the attitude correction (multiplicative, small-angle
        % quat), absorb the bias increment, Joseph-form covariance.
        deltaQuat = [1; 0.5*deltaState(1:3)];
        deltaQuat = deltaQuat/norm(deltaQuat);
        quaternion = quaternionMultiply(quaternion, deltaQuat);
        ekfBiasXY = ekfBiasXY + ...
            deltaState(4:5)*p.estimator.ekf.biasEnabled;
        identityKH = eye(5) - kalmanGain*measurementJacobian;
        ekfCovariance = identityKH*ekfCovariance*identityKH' + ...
            kalmanGain*measurementNoise*kalmanGain';
        % Attitude-covariance floor: a balancing chassis re-tilts at
        % any moment (the plant genuinely violates the constant-tilt
        % assumption every control cycle), so the covariance is not
        % allowed to collapse to the sensor-noise floor -- the Kalman
        % gain would die and the estimate would lag real excursions
        % ~2x (measured).  The floor holds the correction bandwidth
        % near the legacy corrector's ~0.4 s equivalent.
        for diagIndex = 1:3
            ekfCovariance(diagIndex, diagIndex) = max( ...
                ekfCovariance(diagIndex, diagIndex), ...
                p.estimator.ekf.tiltVarFloor);
        end
        rotationWorldFromBody = quaternionToRotation(quaternion);
    end
    end
end
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
% Hold-time gate: the blind band's EDGE is exploitable -- the assembly
% creeping at ~0.025 m/s sits exactly on the filtered-rate threshold,
% flickers the stillness flag, and the drain then hides the real motion
% (observed: a parked payload build crept for seconds with estV ~0.005
% vs true 0.025 until the lean walked off).  Requiring ~1 s of
% CONTINUOUS stillness keeps the phantom-bias cleanup intact while any
% genuine creep keeps interrupting the timer and stays visible to the
% velocity loop.  stillnessHold/drainHoldTime are maintained with the
% ball-rate zeroing above.
if stillnessHold >= drainHoldTime
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

qualifiedPreviously = qualificationTimePrevious >= ...
    p.estimator.biasQualificationTime;
if qualifiedPreviously
    thresholdScale = p.estimator.biasQualificationHysteresis;
else
    thresholdScale = 1;
end

wheelRateLow = max(abs(wheelRate)) <= thresholdScale* ...
    p.estimator.biasWheelRateThreshold;
accelerationNormNominal = abs(accelerationNorm - p.gravity) <= ...
    thresholdScale*p.estimator.biasAccelNormThreshold;
rollPitchRateLow = hypot(gyroBody(1), gyroBody(2)) <= ...
    thresholdScale*p.estimator.biasRollPitchRateThreshold;
yawRateLow = abs(gyroBody(3)) <= thresholdScale* ...
    p.estimator.biasYawRateThreshold;
qualificationCandidate = wheelRateLow && accelerationNormNominal && ...
    rollPitchRateLow && yawRateLow;

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
    relativePositionBody];
diagnostics = [accelerationWorld; gyroBiasZ; ...
    double(biasLearningEnabled); qualificationTime];
end

function matrix = skewMatrix(vector)
matrix = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end

function product = quaternionMultiply(q1, q2)
% Hamilton product q1 ⊗ q2 for [w; x; y; z] quaternions: applies the
% q2 rotation in the q1 frame (error-state injection uses nominal ⊗ delta).
product = [q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4); ...
    q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3); ...
    q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2); ...
    q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)];
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
