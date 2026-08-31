function pose = ballbotPoseFromJoint(jointMeasurement)
%BALLBOTPOSEFROMJOINT Convert position/quaternion sensing to xyz/RPY pose.
% Input: [positionWorld(3); q_WB(4)].

positionWorld = jointMeasurement(1:3);
quaternion = jointMeasurement(4:7);
quaternion = quaternion/max(norm(quaternion), eps);
w = quaternion(1);
x = quaternion(2);
y = quaternion(3);
z = quaternion(4);
rotation = [1 - 2*(y^2 + z^2), 2*(x*y - z*w), 2*(x*z + y*w); ...
    2*(x*y + z*w), 1 - 2*(x^2 + z^2), 2*(y*z - x*w); ...
    2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
pitch = asin(min(max(-rotation(3, 1), -1), 1));
roll = atan2(rotation(3, 2), rotation(3, 3));
yaw = atan2(rotation(2, 1), rotation(1, 1));
pose = [positionWorld; roll; pitch; yaw];
end
