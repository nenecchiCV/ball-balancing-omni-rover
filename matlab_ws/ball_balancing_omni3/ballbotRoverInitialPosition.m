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
position = ballCenterWorld + rotationWorldFromBody* ...
    [0; 0; p.rover.centerAboveBall];
positionWorld = position(component);
end
