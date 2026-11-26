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
                deg2rad(33), AbsTol=1.0e-12);
            testCase.verifyEqual(p.wheel.contactPreload, 5.0e-5, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(p.rover.initialPositionWorld(3), ...
                0.193, AbsTol=1.0e-12);
            testCase.verifyEqual(p.wheel.normalLoadNominal, ...
                p.rover.mass*p.gravity/ ...
                (3*sin(p.wheel.contactLatitude)), RelTol=1.0e-12);
            testCase.verifyEqual(p.wheel.contactTorqueLimit, ...
                p.contact.wheelBall.tractionEstimateFriction* ...
                p.wheel.normalLoadNominal*p.wheel.radius, RelTol=1.0e-12);
            testCase.verifyEqual(p.wheel.commandTorqueLimit, ...
                p.driver.continuousTorqueLimit, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(p.wheel.commandTorqueLimit, ...
                p.wheel.contactTorqueLimit);
        end

        function testFit0521AndMdd3aParameters(testCase)
            p = ballbotParameters;

            testCase.verifyEqual(p.motor.nominalVoltage, 6.0);
            testCase.verifyEqual(p.motor.noLoadSpeed, ...
                210*2*pi/60, AbsTol=1.0e-12);
            testCase.verifyEqual(p.motor.stallCurrent, 3.2);
            testCase.verifyEqual(p.driver.boardCount, 2);
            testCase.verifyEqual(p.driver.continuousCurrent, 3.0);
            testCase.verifyEqual(p.driver.pwmFrequency, 20e3);
            testCase.verifyEqual(p.encoder.pulsesPerOutputRevolution, 341.2);
            testCase.verifyLessThan(p.driver.continuousTorqueLimit, ...
                p.motor.stallTorque);
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
                (p.ball.radius + p.wheel.radius - ...
                p.wheel.contactPreload)*ones(1, 3), ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(sum(geometry.centerFromBall(1:2, :), 2), ...
                zeros(2, 1), AbsTol=1.0e-12);
            testCase.verifyEqual(geometry.centerFromBall(3, :), ...
                (p.ball.radius + p.wheel.radius - ...
                p.wheel.contactPreload)* ...
                sin(p.wheel.contactLatitude)*ones(1, 3), ...
                AbsTol=1.0e-12);
        end

        function testDriveDeadzoneMatchesFirmware(testCase)
            p = ballbotRev1Parameters;

            % FR-16 identified values mirrored from
            % control_params.hpp param_set::drive.
            testCase.verifyEqual(p.drive.deadzoneCompPos, ...
                [0.1817, 0.1596, 0.1786], AbsTol=1.0e-12);
            testCase.verifyEqual(p.drive.deadzoneCompNeg, ...
                [0.1512, 0.1299, 0.1960], AbsTol=1.0e-12);
            testCase.verifyTrue(p.drive.deadzoneCompEnabled);
            % Plant vectors default to the identified comp values
            % (perfect identification baseline for the robustness
            % sweep).
            testCase.verifyEqual(p.drive.deadzonePlantPos, ...
                p.drive.deadzoneCompPos, AbsTol=1.0e-12);
            testCase.verifyEqual(p.drive.deadzonePlantNeg, ...
                p.drive.deadzoneCompNeg, AbsTol=1.0e-12);
        end

        function testGateDriveDeadzoneClip(testCase)
            p = ballbotRev1Parameters;
            logicHigh = p.driver.logicHighVoltage;

            % Wheel 2, positive: below the plant deadzone the winding
            % sees nothing; above it the duty is reduced by dz.
            out = ballbotMdd3aGateDrive(0.05, p, 2);
            testCase.verifyEqual(out(1), 0, AbsTol=1.0e-12);
            out = ballbotMdd3aGateDrive(0.20, p, 2);
            testCase.verifyEqual(out(1), ...
                (0.20 - p.drive.deadzonePlantPos(2))*logicHigh, ...
                AbsTol=1.0e-12);
            % Negative duty uses the Neg vector and asserts REV.
            out = ballbotMdd3aGateDrive(-0.20, p, 2);
            testCase.verifyEqual(out(1), ...
                (0.20 - p.drive.deadzonePlantNeg(2))*logicHigh, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(out(2), logicHigh, AbsTol=1.0e-12);
            % Comp cancellation: servo lifts |duty| by comp, the clip
            % removes the plant dz -- command restored when matched.
            out = ballbotMdd3aGateDrive(0.05 + ...
                p.drive.deadzoneCompPos(2), p, 2);
            testCase.verifyEqual(out(1), 0.05*logicHigh, ...
                AbsTol=1.0e-12);
        end

        function testWheelServoDeadzoneComp(testCase)
            p = ballbotRev1Parameters;
            ctx = zeros(8, 1);
            ctx(7) = 0;   % settleWitness
            ctx(8) = 1;   % enable
            z3 = zeros(3, 1);

            % Zero torque command -> zero duty, comp must NOT create
            % a nonzero duty (firmware: comp inside enable gate only
            % when duty ~= 0).
            out = ballbotWheelServoDutyStep(ctx, z3, z3, 1, p);
            testCase.verifyEqual(out(1:3), z3, AbsTol=1.0e-12);

            % Positive attitude torque -> duty lifted by compPos.
            ctx(1:3) = [0.1; 0; 0];
            outC = ballbotWheelServoDutyStep(ctx, z3, z3, 1, p);
            pNoComp = p;
            pNoComp.drive.deadzoneCompEnabled = false;
            outN = ballbotWheelServoDutyStep(ctx, z3, z3, 1, pNoComp);
            testCase.verifyEqual(outC(1), ...
                outN(1) + p.drive.deadzoneCompPos(1), AbsTol=1.0e-9);

            % Negative torque -> negative duty lifted by compNeg.
            ctx(1:3) = [-0.1; 0; 0];
            outC = ballbotWheelServoDutyStep(ctx, z3, z3, 1, p);
            outN = ballbotWheelServoDutyStep(ctx, z3, z3, 1, pNoComp);
            testCase.verifyEqual(outC(1), ...
                outN(1) - p.drive.deadzoneCompNeg(1), AbsTol=1.0e-9);
        end
    end
end
