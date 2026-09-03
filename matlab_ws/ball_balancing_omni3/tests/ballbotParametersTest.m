classdef ballbotParametersTest < matlab.unittest.TestCase
    %BALLBOTPARAMETERSTEST Verify the 100 mm rigid-ball configuration.

    methods (TestClassSetup)
        function addSourceFolder(testCase)
            sourceFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(sourceFolder));
        end
    end

    methods (Test)
        function testReferenceBallAndDerivedDynamics(testCase)
            p = ballbotParameters;

            testCase.verifyEqual(p.ball.radius, 0.050, AbsTol=1.0e-12);
            testCase.verifyEqual(p.ball.mass, 0.285, AbsTol=1.0e-12);
            testCase.verifyEqual(diag(p.ball.inertia), ...
                4.75e-4*ones(3, 1), AbsTol=1.0e-12);
            testCase.verifyEqual(p.wheel.contactLatitude, ...
                deg2rad(55), AbsTol=1.0e-12);
            testCase.verifyEqual(p.rover.initialPositionWorld(3), ...
                0.175, AbsTol=1.0e-12);
            testCase.verifyEqual(p.wheel.normalLoadNominal, ...
                1.843643204615140, RelTol=1.0e-12);
            testCase.verifyEqual(p.wheel.contactTorqueLimit, ...
                0.033185577683073, RelTol=1.0e-12);
            testCase.verifyEqual(p.wheel.commandTorqueLimit, ...
                p.wheel.contactTorqueLimit, AbsTol=1.0e-12);
        end

        function testWheelGeometryIsSymmetricAndFinite(testCase)
            p = ballbotParameters;
            geometry = p.wheel.geometry;

            testCase.verifyEqual(rank(geometry.ballTorqueFromWheelTorque), 3);
            testCase.verifyTrue(all(isfinite( ...
                geometry.ballTorqueFromWheelTorque), "all"));
            testCase.verifyEqual(vecnorm(geometry.normalBody), ...
                ones(1, 3), AbsTol=1.0e-12);
            testCase.verifyEqual(vecnorm(geometry.centerFromBall), ...
                (p.ball.radius + p.wheel.radius)*ones(1, 3), ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(sum(geometry.centerFromBall(1:2, :), 2), ...
                zeros(2, 1), AbsTol=1.0e-12);
            testCase.verifyEqual(geometry.centerFromBall(3, :), ...
                (p.ball.radius + p.wheel.radius)* ...
                sin(p.wheel.contactLatitude)*ones(1, 3), ...
                AbsTol=1.0e-12);
        end
    end
end
