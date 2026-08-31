function geometry = ballbotWheelGeometry(p)
%BALLBOTWHEELGEOMETRY Construct the symmetric spherical wheel geometry.
% Columns correspond to wheels at azimuths 0, 120, and 240 degrees.

beta = p.wheel.azimuth(:).';
lambda = p.wheel.contactLatitude;

normalBody = [cos(lambda)*cos(beta); ...
    cos(lambda)*sin(beta); ...
    sin(lambda)*ones(1, 3)];
rollingBody = [-sin(beta); cos(beta); zeros(1, 3)];
axleBody = cross(normalBody, rollingBody, 1);

centerFromBall = (p.ball.radius + p.wheel.radius)*normalBody;
bodyFromBall = [0; 0; p.rover.centerAboveBall];
centerFromBody = centerFromBall - bodyFromBall;

geometry.normalBody = normalBody;
geometry.rollingBody = rollingBody;
geometry.axleBody = axleBody;
geometry.centerFromBall = centerFromBall;
geometry.centerFromBody = centerFromBody;
geometry.ballTorqueFromWheelTorque = ...
    (p.ball.radius/p.wheel.radius)*axleBody;
end
