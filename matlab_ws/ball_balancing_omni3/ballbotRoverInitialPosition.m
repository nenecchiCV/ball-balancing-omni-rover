function positionWorld = ballbotRoverInitialPosition(p, component)
%BALLBOTROVERINITIALPOSITION Place a tilted rover about the ball center.

quaternion = ballbotEulerToQuaternion(p.rover.initialEulerWorld);
w = quaternion(1);
x = quaternion(2);
y = quaternion(3);
z = quaternion(4);
rotationWorldFromBody = [ ...
    1 - 2*(y^2 + z^2), 2*(x*y - z*w), 2*(x*z + y*w); ...
    2*(x*y + z*w), 1 - 2*(x^2 + z^2), 2*(y*z - x*w); ...
    2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
ballCenterWorld = [0; 0; p.ball.radius - ...
    p.fullplant.groundStaticDeflection];
% The wheels carry the chassis through penalty contacts: placed at the
% geometric (zero-penetration) height, the assembly free-drops ~N/k at
% t=0 and the asymmetric grab is the startup tilt kick (~0.01-0.02 rad).
% Sink by the static wheel-ball deflection so the contacts start
% preloaded at their rest load.
wheelSink = 0;
if isfield(p.fullplant, "wheelBallStaticDeflection")
    wheelSink = p.fullplant.wheelBallStaticDeflection;
end
position = ballCenterWorld + rotationWorldFromBody* ...
    [0; 0; p.rover.centerAboveBall - wheelSink];
positionWorld = position(component);
end
