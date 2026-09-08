function state = ballbotEstimatorInitialState(p)
%BALLBOTESTIMATORINITIALSTATE Stationary estimator state at initial pose.

if nargin == 0
    quaternion = [1; 0; 0; 0];
else
    quaternion = ballbotEulerToQuaternion(p.rover.initialEulerWorld).';
end
state = [quaternion; zeros(10, 1)];
end
