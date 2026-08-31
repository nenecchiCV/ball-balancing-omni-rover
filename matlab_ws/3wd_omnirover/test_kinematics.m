function tests = test_kinematics
tests = functiontests(localfunctions);
end

function testForwardInverseRoundTrip(testCase)
wheelAngles = deg2rad([0 120 240]);
wheelRadius = 0.024;
wheelCenterRadius = 0.056;
bodyTwistExpected = [0.15; -0.08; 0.4];

wheelSpeed = omni3wdInverseKinematics(bodyTwistExpected, 0, ...
    wheelAngles, wheelRadius, wheelCenterRadius);
bodyTwistActual = omni3wdForwardKinematics(wheelSpeed, ...
    wheelAngles, wheelRadius, wheelCenterRadius);

verifyEqual(testCase, bodyTwistActual, bodyTwistExpected, AbsTol=1e-12);
end

function testWorldCommandRotation(testCase)
wheelAngles = deg2rad([0 120 240]);
wheelRadius = 0.024;
wheelCenterRadius = 0.056;
worldCommand = [0.2; 0; 0];
yaw = pi/2;

wheelSpeed = omni3wdInverseKinematics(worldCommand, yaw, ...
    wheelAngles, wheelRadius, wheelCenterRadius);
bodyTwist = omni3wdForwardKinematics(wheelSpeed, ...
    wheelAngles, wheelRadius, wheelCenterRadius);

verifyEqual(testCase, bodyTwist, [0; -0.2; 0], AbsTol=1e-12);
end

function testCombinedConversionMatrix(testCase)
wheelAngles = deg2rad([0 120 240]);
wheelRadius = 0.024;
wheelCenterRadius = 0.056;
worldCommand = [0.12; 0.06; 0.35];
yaw = deg2rad(35);

[wheelSpeed, worldToWheelMatrix] = omni3wdInverseKinematics( ...
    worldCommand, yaw, wheelAngles, wheelRadius, wheelCenterRadius);

verifySize(testCase, worldToWheelMatrix, [3 3]);
verifyEqual(testCase, wheelSpeed, ...
    worldToWheelMatrix*worldCommand, AbsTol=1e-12);
end
