function results = run_fullplant_regression
%RUN_FULLPLANT_REGRESSION Simulation-based full-plant regression checks.

root = fileparts(fileparts(mfilename("fullpath")));
oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder));
cd(root);
model = "ball_balancing_omni3_multibody_lqi_custom_contact_fullplant";
load_system(model);
set_param(model, "SimulationCommand", "update");
close_system(model, 0);

commands = {[0; 0], [0.02; 0], [0; 0.02], [0.02; 0.02]};
names = ["upright", "x", "y", "xy"];
for k = numel(commands):-1:1
    [metrics, ~] = runFullPlantVerificationCase(commands{k}, 4, 2.5);
    results.(names(k)) = metrics;
    assert(~metrics.fallen, "%s case fell", names(k));
    assert(all(metrics.minimumContactStatus == 1), ...
        "%s case lost contact", names(k));
    assert(all(metrics.minimumNormalForce >= 0), ...
        "%s case produced negative normal force", names(k));
    assert(metrics.maximumMotorTorque <= 0.010001, ...
        "%s case exceeded motor torque limit", names(k));
end
assert(abs(results.x.meanTrackingVelocity(1) - 0.02) < 0.004);
assert(abs(results.y.meanTrackingVelocity(2) - 0.02) < 0.004);
assert(max(abs(results.xy.meanTrackingVelocity - [0.02, 0.02])) < 0.004);
assert(abs(results.x.meanTrackingVelocity(2)) < 0.004);
assert(abs(results.y.meanTrackingVelocity(1)) < 0.004);
end
