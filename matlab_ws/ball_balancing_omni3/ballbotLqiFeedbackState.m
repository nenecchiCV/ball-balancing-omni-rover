function state = ballbotLqiFeedbackState(estimate, wheelRate)
%BALLBOTLQIFEEDBACKSTATE Assemble the feedback vector seen by the LQI.

roll = estimate(1);
pitch = estimate(2);
bodyRate = estimate(4:6);
velocityWorld = estimate(7:8);
state = [velocityWorld; pitch; -roll; bodyRate(2); -bodyRate(1); ...
    bodyRate(3); wheelRate(:)];
end
