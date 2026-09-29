function estimate = ballbotTruthGate(estimate, truthPose, p)
%BALLBOTTRUTHGATE Substitute the estimator output with plant truth.
%   Diagnostic aid: when p.estimator.truthFeedbackEnabled is true, the
%   14-element estimate vector is rebuilt from the logged truth poses
%   [x y z roll pitch yaw] of the rover and the ball, so the balance
%   loop can be exercised with perfect state knowledge.  Rates come
%   from successive quaternions reconstructed from the logged Euler
%   angles; planar velocity from the position difference.  The truth
%   path carries no sensor model, so any residual instability is then
%   attributable to plant/controller physics, not estimation.

if ~isfield(p.estimator, "truthFeedbackEnabled") || ...
        ~p.estimator.truthFeedbackEnabled
    return
end

dt = p.estimator.sampleTime;
persistent previousRover previousBall poseHistory delayInit
if isempty(delayInit)
    poseHistory = zeros(0, 12);
    delayInit = true;
end
rover = truthPose(1:6);
ball = truthPose(7:12);
% Optional artificial latency (p.estimator.truthDelaySteps, default 0):
% substitutes N-sample-old truth, reproducing the estimator path's
% update latency so the value-vs-timing question can be isolated.
delaySteps = p.estimator.truthDelaySteps;
poseHistory = [rover(:).', ball(:).'; poseHistory]; %#ok<AGROW>
if size(poseHistory, 1) > delaySteps + 1
    poseHistory = poseHistory(1:delaySteps + 1, :);
end
delayed = poseHistory(end, :);
rover = delayed(1:6).';
ball = delayed(7:12).';
if isempty(previousRover)
    previousRover = rover;
    previousBall = ball;
end

quaternionNow = eulerToQuaternion(rover(4:6));
quaternionPrev = eulerToQuaternion(previousRover(4:6));
bodyRateWorld = quaternionRate(quaternionPrev, quaternionNow, dt);
rotationWorldFromBody = quaternionToRotation(quaternionNow);
bodyRate = rotationWorldFromBody.'*bodyRateWorld;

ballQuaternionNow = eulerToQuaternion(ball(4:6));
ballQuaternionPrev = eulerToQuaternion(previousBall(4:6));
ballRateWorld = quaternionRate(ballQuaternionPrev, ballQuaternionNow, dt);

velocityWorld = (rover(1:3) - previousRover(1:3))/dt;

rotationWorldFromYaw = [cos(rover(6)), -sin(rover(6)); ...
    sin(rover(6)), cos(rover(6))];
relativePositionBody = rotationWorldFromYaw.'*(rover(1:2) - ball(1:2));

truth = [rover(4:6); bodyRate; velocityWorld(1:2); ...
    ballRateWorld; relativePositionBody; 1.0];
% Optional channel mask (p.estimator.truthChannels, default all 14)
% lets A/B runs substitute only a subset of the estimate, isolating
% which estimated channel drives a divergence.
mask = p.estimator.truthChannels;
if isempty(mask)
    estimate = truth;
else
    estimate(mask) = truth(mask);
end

previousRover = rover;
previousBall = ball;
end

function quaternion = eulerToQuaternion(euler)
roll = euler(1);
pitch = euler(2);
yaw = euler(3);
cr = cos(roll/2);
sr = sin(roll/2);
cp = cos(pitch/2);
sp = sin(pitch/2);
cy = cos(yaw/2);
sy = sin(yaw/2);
quaternion = [cr*cp*cy + sr*sp*sy; ...
    sr*cp*cy - cr*sp*sy; ...
    cr*sp*cy + sr*cp*sy; ...
    cr*cp*sy - sr*sp*cy];
end

function rateWorld = quaternionRate(qPrevious, qNow, dt)
if qPrevious.'*qNow < 0
    qNow = -qNow;
end
delta = quaternionMultiply(qNow, quaternionConjugate(qPrevious));
rateWorld = 2*delta(2:4)/dt;
end

function q = quaternionMultiply(q1, q2)
q = [q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4); ...
     q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3); ...
     q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2); ...
     q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)];
end

function qc = quaternionConjugate(q)
qc = [q(1); -q(2:4)];
end

function rotation = quaternionToRotation(q)
w = q(1);
x = q(2);
y = q(3);
z = q(4);
rotation = [1 - 2*(y^2 + z^2), 2*(x*y - z*w), 2*(x*z + y*w); ...
    2*(x*y + z*w), 1 - 2*(x^2 + z^2), 2*(y*z - x*w); ...
    2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
end
