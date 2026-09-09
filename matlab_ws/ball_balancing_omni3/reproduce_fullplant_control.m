% Reproduce the accepted custom-contact full-plant verification cases.
root = fileparts(mfilename("fullpath"));
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(root);
addpath(fullfile(root, "tests"));

fullplantOperatingPoint = generateFullPlantOperatingPoint;
results = run_fullplant_regression;
save("fullplant_verification_results.mat", "results", "-v7.3");

contact = load("fullplant_contact_validation.mat", "validation");
linearization = load("fullplant_linearization.mat", "fullplantLinearization");
identification = load("fullplant_identification_evidence.mat", "fullplantEvidence");
design = load("fullplant_controller_design.mat", "fullplantDesign");
assert(all(contact.validation.contactEnd == 1));
assert(contact.validation.anisotropyRatio > 30);
assert(linearization.fullplantLinearization.order == 0);
assert(identification.fullplantEvidence.analyticLinearizationWasZero);
assert(design.fullplantDesign.hierarchy.adopted);
disp(results)
