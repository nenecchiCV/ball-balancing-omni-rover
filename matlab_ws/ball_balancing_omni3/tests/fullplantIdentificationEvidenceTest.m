classdef fullplantIdentificationEvidenceTest < matlab.unittest.TestCase
    %FULLPLANTIDENTIFICATIONEVIDENCETEST Validate saved analysis evidence.

    properties (TestParameter)
        FrfIndex = {1, 2, 3}
    end

    methods (TestClassSetup)
        function addSourceFolder(testCase)
            sourceFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(sourceFolder));
        end
    end

    methods (Test)
        function testLinearizationIsFinite(testCase)
            evidence = loadEvidence;
            matrices = [evidence.linearPlant.A(:); ...
                evidence.linearPlant.B(:); evidence.linearPlant.C(:); ...
                evidence.linearPlant.D(:)];
            testCase.verifyTrue(all(isfinite(matrices)));
            testCase.verifySize(evidence.linearPlant, [10, 3]);
        end

        function testFrequencyResponseIsFinite(testCase, FrfIndex)
            evidence = loadEvidence;
            response = evidence.frf{FrfIndex}.ResponseData;
            testCase.verifyTrue(all(isfinite(response), "all"));
            testCase.verifySize(evidence.frf{FrfIndex}, [10, 3]);
        end

        function testUnusableOperatingPointRemainsExplicit(testCase)
            evidence = loadEvidence;
            violation = ...
                evidence.trimReport.OptimizationOutput.constrviolation;
            testCase.verifyGreaterThan(violation, 1.0e-6);
            testCase.verifyGreaterThan( ...
                min(evidence.relativeMismatch20To100RadPerSec), 1.0);
        end
    end
end

function evidence = loadEvidence
sourceFolder = fileparts(fileparts(mfilename("fullpath")));
data = load(fullfile(sourceFolder, ...
    "fullplant_identification_evidence.mat"), "fullplantEvidence");
evidence = data.fullplantEvidence;
end
