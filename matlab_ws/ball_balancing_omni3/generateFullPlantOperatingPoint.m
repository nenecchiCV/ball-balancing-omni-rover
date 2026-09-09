function fullplantOperatingPoint = generateFullPlantOperatingPoint
%GENERATEFULLPLANTOPERATINGPOINT Rebuild the custom-contact snapshot point.

model = "ball_balancing_omni3_multibody_lqi_custom_contact_fullplant";
p = ballbotFullPlantParameters;
p.command.enable = false;
p.command.profileEnabled = false;
assignin("base", "ballbotParams", p);
op = findop(model, 0.1);
initialState = getstatestruct(op);
in = Simulink.SimulationInput(model);
in = in.setVariable("ballbotParams", p);
in = in.setInitialState(initialState);
in = in.setModelParameter("StopTime", "0.1");
out = sim(in);
wheel = out.wheelDisplacement;
wheelSpeed = (wheel.Data(end, :) - wheel.Data(end - 1, :))/ ...
    (wheel.Time(end) - wheel.Time(end - 1));
residual = max(abs([out.imuMeasurement.Data(end, 4:6), ...
    out.estimatedState.Data(end, 7:8), wheelSpeed]));
metrics.rollPitch = out.roverPose.Data(end, 4:5);
metrics.bodyAngularRate = out.imuMeasurement.Data(end, 4:6);
metrics.wheelSpeed = wheelSpeed;
metrics.wheelSpeedCommand = out.wheelSpeedCommand.Data(end, :);
metrics.motorTorque = out.wheelTorqueCommand.Data(end, :);
metrics.normalForce = [out.wheel1NormalForce.Data(end), ...
    out.wheel2NormalForce.Data(end), out.wheel3NormalForce.Data(end)];
metrics.contacts = out.contactStatus.Data(end, :);
fullplantOperatingPoint = struct("model", model, ...
    "generatedOn", datetime("now"), "op", op, ...
    "initialState", initialState, "method", "0.1 s snapshot", ...
    "defaultTrimFeasible", false, ...
    "defaultTrimMaximumDerivative", 0.016167, ...
    "reapplicationResidual", residual, "metrics", metrics);
save("fullplant_operating_point.mat", "fullplantOperatingPoint", "-v7.3");
end
