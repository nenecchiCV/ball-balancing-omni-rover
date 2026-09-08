function quaternion = ballbotEulerToQuaternion(eulerAngles)
%BALLBOTEULERTOQUATERNION Convert XYZ roll/pitch/yaw to scalar-first form.

roll = eulerAngles(1);
pitch = eulerAngles(2);
yaw = eulerAngles(3);
cr = cos(roll/2);
sr = sin(roll/2);
cp = cos(pitch/2);
sp = sin(pitch/2);
cy = cos(yaw/2);
sy = sin(yaw/2);
quaternion = [cr*cp*cy + sr*sp*sy, ...
    sr*cp*cy - cr*sp*sy, ...
    cr*sp*cy + sr*cp*sy, ...
    cr*cp*sy - sr*sp*cy];
end
