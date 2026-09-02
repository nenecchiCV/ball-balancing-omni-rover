function ready = ballbotYawBiasStartupGuard( ...
    previousReady, correctedYawRate, biasLearningEnabled, p)
%BALLBOTYAWBIASSTARTUPGUARD Latch completion of startup yaw calibration.
% Roll/pitch balance remains active before this latch sets; only yaw
% actuation is inhibited. The latch is reset by estimator/controller reset.

ready = logical(previousReady) || (logical(biasLearningEnabled) && ...
    abs(correctedYawRate) <= p.controller.yawBiasReadyRateThreshold);
end
