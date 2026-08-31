function [nextState, estimate, diagnostics] = ballbotEstimatorStep( ...
    previousState, imu, wheelRate, p)
%BALLBOTESTIMATORSTEP Ideal-IMU/derived-wheel-rate observer update.
% State: [q_WB(4); v_WB(3); omegaBall_W(3); roverMinusBall_B_xy(2)].
% Estimate: [roll; pitch; yaw; p; q; r; vx_W; vy_W; omegaBall_W(3);
%            roverMinusBall_B_xy(2); contactConfidence].

dt = p.estimator.sampleTime;
quaternion = previousState(1:4);
velocityWorld = previousState(5:7);
ballRateWorldPrevious = previousState(8:10);
relativePositionBodyPrevious = previousState(11:12);

specificForceBody = imu(1:3);
gyroBody = imu(4:6);
rotationWorldFromBody = quaternionToRotation(quaternion);

accelerationNorm = norm(specificForceBody);
if abs(accelerationNorm - p.gravity) <= p.estimator.accelNormGate
    measuredUpBody = specificForceBody/max(accelerationNorm, eps);
    predictedUpBody = rotationWorldFromBody'*[0; 0; 1];
    correction = p.estimator.attitudeCorrectionGain* ...
        cross(measuredUpBody, predictedUpBody);
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
filterWeight = dt/(p.estimator.ballRateTimeConstant + dt);
ballRateWorld = ballRateWorldPrevious + ...
    filterWeight*(ballRateKinematic - ballRateWorldPrevious);

ballVelocityWorld = groundRollingMatrix*ballRateWorld;
relativeVelocityWorld = velocityWorld - ballVelocityWorld;
yaw = rotationToEuler(rotationWorldFromBody);
rotationWorldFromYaw = [cos(yaw(3)), -sin(yaw(3)); ...
    sin(yaw(3)), cos(yaw(3))];
relativePositionWorldPrevious = rotationWorldFromYaw* ...
    relativePositionBodyPrevious;
relativePositionWorld = p.estimator.relativePositionLeak* ...
    relativePositionWorldPrevious + dt*relativeVelocityWorld(1:2);
relativePositionBody = rotationWorldFromYaw'*relativePositionWorld;

residual = kinematicMatrix*ballRateWorld - kinematicRightSide;
residualRootMeanSquare = sqrt(mean(residual.^2));
contactConfidence = exp(-(residualRootMeanSquare/ ...
    p.estimator.contactResidualScale)^2);

nextState = [quaternion; velocityWorld; ballRateWorld; ...
    relativePositionBody];
estimate = [yaw; gyroBody; velocityWorld(1:2); ballRateWorld; ...
    relativePositionBody; contactConfidence];
diagnostics = [accelerationWorld; residual; residualRootMeanSquare];
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
