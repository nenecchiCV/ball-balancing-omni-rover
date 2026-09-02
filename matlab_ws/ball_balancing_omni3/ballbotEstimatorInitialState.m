function state = ballbotEstimatorInitialState
%BALLBOTESTIMATORINITIALSTATE Upright, stationary estimator state.

state = [1; 0; 0; 0; zeros(10, 1)];
end
