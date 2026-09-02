classdef ballbotEstimatorStepTest < matlab.unittest.TestCase
    %BALLBOTESTIMATORSTEPTEST Yaw-gyro bias observer unit tests.

    methods (TestClassSetup)
        function addSourceFolder(testCase)
            sourceFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(sourceFolder));
        end
    end

    methods (Test)
        function testWheelRateBackwardDifference(testCase)
            wheelDisplacement = [0.01; -0.02; 0.03];
            previousWheelDisplacement = zeros(3, 1);

            wheelRate = ballbotWheelRateFromDisplacement( ...
                wheelDisplacement, previousWheelDisplacement, 0.005);

            testCase.verifyEqual(wheelRate, [2; -4; 6], ...
                AbsTol=1.0e-12);
        end

        function testImuYawBiasInjection(testCase)
            p = ballbotParameters;
            p.simulation.imuYawBias = 0.02;
            jointMeasurement = [1; zeros(9, 1)];

            imu = ballbotImuFromJoint(jointMeasurement, p);

            testCase.verifyEqual(imu, ...
                [0; 0; p.gravity; 0; 0; 0.02], AbsTol=1.0e-12);
        end

        function testStationaryZeroBias(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0);

            [state, estimate, diagnostics] = ...
                ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, 2000);

            testCase.verifyEqual(state(13), 0, AbsTol=1.0e-12);
            testCase.verifyEqual(estimate(4:6), zeros(3, 1), ...
                AbsTol=1.0e-12);
            testCase.verifyGreaterThan(estimate(14), 0.99);
            testCase.verifyTrue(all(isfinite([state; estimate; diagnostics])));
        end

        function testStationaryKnownBiasConvergesWithinFifteenSeconds(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            injectedBias = 0.02;
            imu = ballbotEstimatorStepTest.stationaryImu(p, injectedBias);
            sampleCount = round(15/p.estimator.sampleTime);

            [state, ~, diagnostics] = ...
                ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, sampleCount);

            testCase.verifyLessThan(abs(state(13) - injectedBias), 0.002);
            testCase.verifyEqual(diagnostics(9), 1, AbsTol=0);
        end

        function testQualificationDwellPreventsLearning(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.02);
            sampleCount = round(p.estimator.biasQualificationTime/ ...
                p.estimator.sampleTime);

            [state, ~, diagnostics] = ...
                ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, sampleCount);

            testCase.verifyEqual(state(13), 0, AbsTol=1.0e-12);
            testCase.verifyEqual(diagnostics(9), 0, AbsTol=0);
            testCase.verifyEqual(state(14), ...
                p.estimator.biasQualificationTime, AbsTol=1.0e-12);
        end

        function testWheelMotionPreventsLearning(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.02);
            wheelRate = 2*p.estimator.biasWheelRateThreshold*ones(3, 1);

            [state, ~, diagnostics] = ...
                ballbotEstimatorStepTest.runSamples( ...
                state, imu, wheelRate, p, 1000);

            testCase.verifyEqual(state(13), 0, AbsTol=1.0e-12);
            testCase.verifyEqual(state(14), 0, AbsTol=1.0e-12);
            testCase.verifyEqual(diagnostics(9), 0, AbsTol=0);
        end

        function testAccelerationNormAnomalyPreventsLearning(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.02);
            imu(3) = p.gravity + 2*p.estimator.biasAccelNormThreshold;

            [state, ~, diagnostics] = ...
                ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, 1000);

            testCase.verifyEqual(state(13), 0, AbsTol=1.0e-12);
            testCase.verifyEqual(state(14), 0, AbsTol=1.0e-12);
            testCase.verifyEqual(diagnostics(9), 0, AbsTol=0);
        end

        function testLowContactConfidencePreventsLearning(testCase)
            p = ballbotParameters;
            p.estimator.kinematicRegularization = 1;
            p.estimator.contactResidualScale = 1.0e-6;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.02);

            [state, estimate, diagnostics] = ...
                ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, 1000);

            testCase.verifyLessThan(estimate(14), ...
                p.estimator.biasContactConfidenceThreshold);
            testCase.verifyEqual(state(13), 0, AbsTol=1.0e-12);
            testCase.verifyEqual(diagnostics(9), 0, AbsTol=0);
        end

        function testBiasSaturatesAtConfiguredLimit(testCase)
            p = ballbotParameters;
            p.estimator.biasMaximum = 0.01;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.04);

            state = ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, 5000);

            testCase.verifyLessThanOrEqual(abs(state(13)), ...
                p.estimator.biasMaximum + 1.0e-12);
            testCase.verifyEqual(state(13), p.estimator.biasMaximum, ...
                AbsTol=1.0e-8);
        end

        function testLearningResumesAfterMotion(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.02);
            dwellSamples = round(p.estimator.biasQualificationTime/ ...
                p.estimator.sampleTime) + 200;
            wheelRate = 2*p.estimator.biasWheelRateThreshold*ones(3, 1);

            state = ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, dwellSamples);
            biasBeforeMotion = state(13);
            state = ballbotEstimatorStepTest.runSamples( ...
                state, imu, wheelRate, p, 200);
            biasAfterMotion = state(13);
            state = ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, dwellSamples);

            testCase.verifyGreaterThan(biasBeforeMotion, 0);
            testCase.verifyEqual(biasAfterMotion, biasBeforeMotion, ...
                AbsTol=1.0e-12);
            testCase.verifyGreaterThan(state(13), biasAfterMotion);
        end

        function testInterfaceAndControllerRegression(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0);

            [state, estimate, diagnostics] = ballbotEstimatorStep( ...
                state, imu, zeros(3, 1), p);
            [wheelTorque, ~, mode] = ballbotControlStep( ...
                estimate, zeros(3, 1), zeros(2, 1), true, p);

            testCase.verifySize(state, [14, 1]);
            testCase.verifySize(estimate, [14, 1]);
            testCase.verifySize(diagnostics, [10, 1]);
            testCase.verifyEqual(estimate(1:13), zeros(13, 1), ...
                AbsTol=1.0e-10);
            testCase.verifyGreaterThan(estimate(14), 0.99);
            testCase.verifyEqual(wheelTorque, zeros(3, 1), ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(mode, uint8(1));
        end

        function testEstimatorUpdatePacking(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0);

            [output, diagnostics] = ballbotEstimatorUpdate( ...
                state, imu, zeros(3, 1), p);

            testCase.verifySize(output, [31, 1]);
            testCase.verifySize(diagnostics, [10, 1]);
            testCase.verifyEqual(output(29:31), diagnostics(8:10), ...
                AbsTol=1.0e-12);
        end

        function testYawStartupGuardLatchesAfterConvergence(testCase)
            p = ballbotParameters;

            readyBeforeConvergence = ballbotYawBiasStartupGuard(false, ...
                2*p.controller.yawBiasReadyRateThreshold, true, p);
            readyAfterConvergence = ballbotYawBiasStartupGuard(false, ...
                0.5*p.controller.yawBiasReadyRateThreshold, true, p);
            readyDuringMotion = ballbotYawBiasStartupGuard( ...
                readyAfterConvergence, p.controller.maxYawRate, false, p);

            testCase.verifyFalse(readyBeforeConvergence);
            testCase.verifyTrue(readyAfterConvergence);
            testCase.verifyTrue(readyDuringMotion);
        end

        function testControllerInhibitsYawUntilBiasIsReady(testCase)
            p = ballbotParameters;
            estimate = zeros(14, 1);
            estimate(6) = 0.02;
            estimate(14) = 1;
            biasDiagnostics = [0; 1; p.estimator.biasQualificationTime];

            outputNotReady = ballbotControllerUpdate(estimate, ...
                zeros(3, 1), zeros(2, 1), true, false, ...
                biasDiagnostics, p);
            outputReady = ballbotControllerUpdate(estimate, ...
                zeros(3, 1), zeros(2, 1), true, true, ...
                biasDiagnostics, p);
            yawCommand = [0; 0; 0.2];
            outputCommanded = ballbotControllerUpdate(estimate, ...
                yawCommand, zeros(2, 1), true, false, ...
                biasDiagnostics, p);

            testCase.verifySize(outputNotReady, [7, 1]);
            testCase.verifyEqual(outputNotReady(1:3), zeros(3, 1), ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(outputNotReady(7), 0, AbsTol=0);
            testCase.verifyGreaterThan(norm(outputReady(1:3)), 0);
            testCase.verifyEqual(outputReady(7), 1, AbsTol=0);
            testCase.verifyGreaterThan(norm(outputCommanded(1:3)), 0);
            testCase.verifyEqual(outputCommanded(7), 0, AbsTol=0);
        end

        function testPostLearningYawDrift(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.02);

            [state, estimateBefore] = ...
                ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, ...
                round(15/p.estimator.sampleTime));
            [~, estimateAfter] = ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, ...
                round(10/p.estimator.sampleTime));
            yawDriftDegrees = rad2deg( ...
                abs(estimateAfter(3) - estimateBefore(3)));

            testCase.verifyLessThan(yawDriftDegrees, 0.5);
        end

        function testBiasHeldDuringMotion(testCase)
            p = ballbotParameters;
            state = ballbotEstimatorInitialState;
            imu = ballbotEstimatorStepTest.stationaryImu(p, 0.02);

            state = ballbotEstimatorStepTest.runSamples( ...
                state, imu, zeros(3, 1), p, ...
                round(15/p.estimator.sampleTime));
            biasBeforeMotion = state(13);
            wheelRate = 2*p.estimator.biasWheelRateThreshold*ones(3, 1);
            state = ballbotEstimatorStepTest.runSamples( ...
                state, imu, wheelRate, p, ...
                round(5/p.estimator.sampleTime));

            testCase.verifyLessThan(abs(state(13) - biasBeforeMotion), 0.002);
        end
    end

    methods (Static, Access = private)
        function imu = stationaryImu(p, yawBias)
            imu = [0; 0; p.gravity; 0; 0; yawBias];
        end

        function [state, estimate, diagnostics] = runSamples( ...
                state, imu, wheelRate, p, sampleCount)
            estimate = zeros(14, 1);
            diagnostics = zeros(10, 1);
            for sampleIndex = 1:sampleCount
                [state, estimate, diagnostics] = ballbotEstimatorStep( ...
                    state, imu, wheelRate, p);
            end
        end
    end
end
