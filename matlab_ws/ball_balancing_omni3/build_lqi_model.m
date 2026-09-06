function build_lqi_model
%BUILD_LQI_MODEL Create an LQI variant from the cascaded controller model.

sourceModel = "ball_balancing_omni3_multibody_cascade";
targetModel = "ball_balancing_omni3_multibody_lqi";

open_system(sourceModel);
if bdIsLoaded(targetModel)
    close_system(targetModel, 0);
end
save_system(sourceModel, targetModel + ".slx");
open_system(targetModel);

controller = targetModel + "/Controller";
oldBlock = controller + "/CascadeController";
oldPosition = get_param(oldBlock, "Position");
lineHandles = get_param(oldBlock, "LineHandles");
delete_line(lineHandles.Inport(lineHandles.Inport ~= -1));
delete_line(lineHandles.Outport(lineHandles.Outport ~= -1));
delete_block(oldBlock);

sourceBlock = "ball_balancing_omni3_multibody/Controller/ControllerUpdate";
newBlock = controller + "/LqiController";
add_block(sourceBlock, newBlock, "Position", oldPosition, ...
    "MATLABFcn", ...
    ["ballbotLqiControllerUpdate(u(1:14),u(15:17),u(18:19)," ...
    "u(20),u(21),u(22:24),u(25:27),ballbotParams)"], ...
    "OutputDimensions", "7");
add_line(controller, "ControlInputs/1", "LqiController/1", ...
    "autorouting", "on");
for destination = ["SelectNextVelocityIntegral", "SelectYawBiasReady", ...
        "SelectMode", "SelectRequestedTorque"]
    add_line(controller, "LqiController/1", destination + "/1", ...
        "autorouting", "on");
end

set_param(targetModel, "PreLoadFcn", ...
    ["ballbotParams = ballbotParameters; " ...
    "ballbotParams.controller.minimumContactConfidence = 0.10;"]);
ballbotParams = ballbotParameters;
ballbotParams.controller.minimumContactConfidence = 0.10;
assignin("base", "ballbotParams", ballbotParams);
save_system(targetModel);
set_param(targetModel, "SimulationCommand", "update");
save_system(targetModel);
end
