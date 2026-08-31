function imu = ballbotImuFromJoint(jointMeasurement, p)
%BALLBOTIMUFROMJOINT Convert 6-DOF Joint sensing into an ideal 6-axis IMU.
% Input: [q_WB(4); accelerationWorld(3); angularVelocityBody(3)].

quaternion = jointMeasurement(1:4);
quaternion = quaternion/max(norm(quaternion), eps);
rotationWorldFromBody = quaternionToRotation(quaternion);
accelerationWorld = jointMeasurement(5:7);
angularVelocityBody = jointMeasurement(8:10);
imu = ballbotIdealImu(rotationWorldFromBody, accelerationWorld, ...
    angularVelocityBody, p);
end

function rotation = quaternionToRotation(quaternion)
w = quaternion(1);
x = quaternion(2);
y = quaternion(3);
z = quaternion(4);
rotation = [1 - 2*(y^2 + z^2), 2*(x*y - z*w), 2*(x*z + y*w); ...
    2*(x*y + z*w), 1 - 2*(x^2 + z^2), 2*(y*z - x*w); ...
    2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
end
