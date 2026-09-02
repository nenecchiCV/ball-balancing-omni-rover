function wheelRate = ballbotWheelRateFromDisplacement( ...
    wheelDisplacement, previousWheelDisplacement, sampleTime)
%BALLBOTWHEELRATEFROMDISPLACEMENT Compute wheel rate by backward difference.

wheelRate = (wheelDisplacement(:) - previousWheelDisplacement(:))/ ...
    sampleTime;
end
