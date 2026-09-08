function tests = fullplantDesignArtifactsTest
%FULLPLANTDESIGNARTIFACTSTEST Verify Full Multibody design artifacts.
tests = functiontests(localfunctions);
end

function testModelAndEvidenceExist(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
verifyTrue(testCase, isfile(fullfile(root, ...
    "ball_balancing_omni3_multibody_lqi_fullplant.slx")));
verifyTrue(testCase, isfile(fullfile(root, "fullplant_operating_point.mat")));
verifyTrue(testCase, isfile(fullfile(root, "fullplant_linearization.mat")));
end

function testLinearModelIsFinite(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
data = load(fullfile(root, "fullplant_linearization.mat"), "sysPlant");
verifySize(testCase, data.sysPlant.B, [38 3]);
verifySize(testCase, data.sysPlant.C, [10 38]);
verifyTrue(testCase, all(isfinite(data.sysPlant.A), "all"));
verifyTrue(testCase, all(isfinite(data.sysPlant.B), "all"));
verifyTrue(testCase, all(isfinite(data.sysPlant.C), "all"));
verifyTrue(testCase, all(isfinite(data.sysPlant.D), "all"));
end

function testUnvalidatedGainIsNotSilentlyAdopted(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
verifyFalse(testCase, isfile(fullfile(root, "fullplant_lqi_design.mat")));
end

function testUnconstrainedContactEnvelope(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root)
p = ballbotFullPlantParameters;
verifyEqual(testCase, p.wheel.contactPreload, 2.0e-4, ...
    "AbsTol", 1.0e-12);
verifyEqual(testCase, p.controller.recoveryTilt, deg2rad(17), ...
    "AbsTol", 1.0e-12);
end

function testHighCenterOfMassPayload(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root)
p = ballbotFullPlantParameters;
verifyEqual(testCase, p.fullplant.payload.mass, 0.50, ...
    "AbsTol", 1.0e-12);
verifyEqual(testCase, p.fullplant.payload.length, 0.30, ...
    "AbsTol", 1.0e-12);
verifyEqual(testCase, p.fullplant.payload.width, 0.020, ...
    "AbsTol", 1.0e-12);
verifyGreaterThan(testCase, p.fullplant.payload.comAboveChassis, 0.075);
end
