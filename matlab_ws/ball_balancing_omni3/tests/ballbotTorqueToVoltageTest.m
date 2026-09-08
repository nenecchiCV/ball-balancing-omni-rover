classdef ballbotTorqueToVoltageTest < matlab.unittest.TestCase
    %BALLBOTTORQUETOVOLTAGETEST Verify open-loop torque-voltage inversion.

    methods (TestClassSetup)
        function addSourceFolder(testCase)
            sourceFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(sourceFolder));
        end
    end

    methods (Test)
        function testTorqueConversionAtZeroSpeed(testCase)
            p = ballbotParameters;
            torqueCommand = [0.01; -0.02; 0.03];

            voltage = ballbotTorqueToVoltage( ...
                torqueCommand, zeros(3, 1), 1, p);

            expected = p.motor.armatureResistance/ ...
                p.motor.torqueConstant*torqueCommand;
            testCase.verifyEqual(voltage, expected, AbsTol=1.0e-12);
        end

        function testBackEmfAndFrictionCompensation(testCase)
            p = ballbotParameters;
            wheelSpeed = [1; -2; 3];

            voltage = ballbotTorqueToVoltage( ...
                zeros(3, 1), wheelSpeed, 1, p);

            expectedGain = p.motor.backEmfConstant + ...
                p.motor.armatureResistance*p.motor.viscousFriction/ ...
                p.motor.torqueConstant;
            testCase.verifyEqual(voltage, expectedGain*wheelSpeed, ...
                AbsTol=1.0e-12);
        end

        function testVoltageSaturationAndDisabledMode(testCase)
            p = ballbotParameters;
            voltageLimit = p.driver.maxDutyCycle*p.driver.supplyVoltage;
            highSpeed = 1000*ones(3, 1);

            saturatedVoltage = ballbotTorqueToVoltage( ...
                p.wheel.commandTorqueLimit*ones(3, 1), ...
                highSpeed, 1, p);
            disabledVoltage = ballbotTorqueToVoltage( ...
                p.wheel.commandTorqueLimit*ones(3, 1), ...
                highSpeed, 0, p);

            testCase.verifyEqual(saturatedVoltage, ...
                voltageLimit*ones(3, 1), AbsTol=1.0e-12);
            testCase.verifyEqual(disabledVoltage, zeros(3, 1), ...
                AbsTol=1.0e-12);
        end

        function testLqiOutputIsBoundedWheelSpeedCommand( ...
                testCase)
            p = ballbotParameters;
            estimate = zeros(14, 1);
            estimate(2) = deg2rad(2);
            estimate(14) = 1;
            biasDiagnostics = [0; 0; 0];

            outputStopped = ballbotLqiControllerUpdate(estimate, ...
                zeros(3, 1), zeros(2, 1), true, false, ...
                biasDiagnostics, zeros(3, 1), p);
            outputRotating = ballbotLqiControllerUpdate(estimate, ...
                zeros(3, 1), zeros(2, 1), true, false, ...
                biasDiagnostics, [10; -20; 30], p);

            % The outer LQI outputs wheel-speed commands. Wheel rate is an
            % explicit feedback state, so rotating and stopped responses
            % should differ; torque limiting belongs to the inner PI/motor.
            testCase.verifyNotEqual(outputStopped(1:3), ...
                outputRotating(1:3));
            testCase.verifyLessThanOrEqual( ...
                max(abs(outputStopped(1:3))), ...
                p.motor.speedCommandLimit + 1.0e-12);
            testCase.verifyLessThanOrEqual( ...
                max(abs(outputRotating(1:3))), ...
                p.motor.speedCommandLimit + 1.0e-12);
        end
    end
end
