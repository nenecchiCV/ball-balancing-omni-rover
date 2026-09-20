classdef fullplantIdentificationEvidenceTest < matlab.unittest.TestCase
    %FULLPLANTIDENTIFICATIONEVIDENCETEST Validate saved analysis evidence.

    properties (TestParameter)
        TrialIndex = {1, 2, 3, 4, 5, 6}
    end

    methods (TestClassSetup)
        function addSourceFolder(testCase)
            sourceFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(sourceFolder));
        end
    end

    methods (Test)
        function testIdentifiedModelIsFinite(testCase)
            evidence = loadEvidence;
            testCase.verifySize(evidence.A, [4, 4]);
            testCase.verifySize(evidence.B, [4, 1]);
            testCase.verifyTrue(all(isfinite(evidence.A), "all"));
            testCase.verifyTrue(all(isfinite(evidence.B), "all"));
            testCase.verifyNumElements(evidence.stateDefinition, 4);
        end

        function testTrialsAreFinite(testCase, TrialIndex)
            evidence = loadEvidence;
            trial = evidence.trials(TrialIndex);
            testCase.verifyTrue(all(isfinite(trial.state), "all"));
            testCase.verifyTrue(all(isfinite(trial.input), "all"));
            testCase.verifyTrue(all(isfinite(trial.time), "all"));
            testCase.verifyTrue(all(trial.minimumContact >= 0));
            testCase.verifyGreaterThan(min(trial.maxTiltDeg), 0);
        end

        function testFitPercentIsRecorded(testCase)
            evidence = loadEvidence;
            testCase.verifyTrue(all(isfinite(evidence.fitPercent)));
            testCase.verifyTrue(all(evidence.fitPercent > 50));
        end

        function testAnalyticLinearizationRejectionIsExplicit(testCase)
            evidence = loadEvidence;
            testCase.verifyTrue( ...
                evidence.analyticLinearizationWasZero);
            testCase.verifyTrue(contains( ...
                evidence.analyticAdvisorBlock, "SpeedPiAndDcMotor"));
        end
    end
end

function evidence = loadEvidence
sourceFolder = fileparts(fileparts(mfilename("fullpath")));
data = load(fullfile(sourceFolder, ...
    "fullplant_identification_evidence.mat"), "fullplantEvidence");
evidence = data.fullplantEvidence;
end
