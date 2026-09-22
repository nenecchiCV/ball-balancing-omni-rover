function tests = fullplantDesignArtifactsTest
%FULLPLANTDESIGNARTIFACTSTEST Verify Full Multibody design artifacts.
tests = functiontests(localfunctions);
end

function testModelAndEvidenceExist(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
verifyTrue(testCase, isfile(fullfile(root, ...
    "ball_balancing_omni3_multibody_lqi_custom_contact_fullplant.slx")));
verifyTrue(testCase, isfile(fullfile(root, "fullplant_operating_point.mat")));
verifyTrue(testCase, isfile(fullfile(root, "fullplant_linearization.mat")));
end

function testLinearModelIsFinite(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
data = load(fullfile(root, "fullplant_linearization.mat"), ...
    "fullplantLinearization");
verifyClass(testCase, data.fullplantLinearization.sys, "ss");
verifySize(testCase, data.fullplantLinearization.sys.D, [10 3]);
verifyTrue(testCase, all(isfinite( ...
    data.fullplantLinearization.sys.D), "all"));
end

function testDegenerateAnalyticLinearizationIsNotAdopted(testCase)
root = fileparts(fileparts(mfilename("fullpath")));
data = load(fullfile(root, "fullplant_linearization.mat"), ...
    "fullplantLinearization", "sysPlant");
verifyFalse(testCase, data.fullplantLinearization.adopted);
verifyTrue(testCase, contains( ...
    data.fullplantLinearization.reason, "SpeedPiAndDcMotor"));
verifySize(testCase, data.sysPlant.D, [10 3]);
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
% With the tiered frame structure the unladen chassis COM sits below the
% frame origin (0.125 m), so the payload contribution measured against
% centerAboveBall is smaller than the original lumped-mass estimate.
verifyGreaterThan(testCase, p.fullplant.payload.comAboveChassis, 0.030);
verifyEqual(testCase, p.rover.comAboveBall, ...
    p.rover.centerAboveBall + p.fullplant.payload.comAboveChassis, ...
    "AbsTol", 1.0e-12);
verifyEqual(testCase, p.wheel.normalLoadNominal, ...
    p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude)), ...
    "AbsTol", 1.0e-12);
verifyGreaterThan(testCase, p.wheel.contactTorqueLimit, 0.077);
baseline = ballbotParameters;
verifyNotEqual(testCase, p.controller.lqi.stateGain, ...
    baseline.controller.lqi.stateGain);
verifyTrue(testCase, p.controller.lqi.augmentedStabilizable);
end
