function wheelRate = ballbotWheelRateFromDisplacement( ...
    wheelDisplacement, previousWheelDisplacement, sampleTime)
%BALLBOTWHEELRATEFROMDISPLACEMENT Differentiate wheel encoder positions.

arguments
    wheelDisplacement (3,1) double
    previousWheelDisplacement (3,1) double
    sampleTime (1,1) double {mustBePositive}
end

wheelRate = (wheelDisplacement - previousWheelDisplacement)/sampleTime;
end
