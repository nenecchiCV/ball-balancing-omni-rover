function summary = review_fullplant_design
%REVIEWFULLPLANTDESIGN Review saved Full Multibody design evidence.

root = fileparts(mfilename("fullpath"));
opData = load(fullfile(root, "fullplant_operating_point.mat"));
linData = load(fullfile(root, "fullplant_linearization.mat"));
system = linData.sysPlant;

summary = struct;
summary.operatingPointReport = opData.rep5.TerminationString;
summary.stateCount = size(system.A, 1);
summary.inputCount = size(system.B, 2);
summary.outputCount = size(system.C, 1);
summary.isFinite = all(isfinite(system.A), "all") && ...
    all(isfinite(system.B), "all") && ...
    all(isfinite(system.C), "all") && all(isfinite(system.D), "all");
summary.maximumPoleMagnitude = max(abs(pole(system)));
summary.controllabilityRank = rank(ctrb(system.A, system.B));
summary.observabilityRank = rank(obsv(system.A, system.C));
disp(summary)
end
