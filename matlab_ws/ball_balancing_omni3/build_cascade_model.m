function build_cascade_model
%BUILD_CASCADE_MODEL Create the block-based cascaded controller variant.

sourceModel = "ball_balancing_omni3_multibody";
targetModel = "ball_balancing_omni3_multibody_cascade";

open_system(sourceModel);
if bdIsLoaded(targetModel)
    close_system(targetModel, 0);
end
save_system(sourceModel, targetModel + ".slx");
open_system(targetModel);

controller = targetModel + "/Controller";
oldBlock = controller + "/ControllerUpdate";
oldPosition = get_param(oldBlock, "Position");
lineHandles = get_param(oldBlock, "LineHandles");
delete_line(lineHandles.Inport(lineHandles.Inport ~= -1));
delete_line(lineHandles.Outport(lineHandles.Outport ~= -1));
delete_block(oldBlock);
add_block("simulink/Ports & Subsystems/Subsystem", ...
    controller + "/CascadeController", "Position", oldPosition);
buildCascadeSubsystem(controller + "/CascadeController");

% Reconnect the replacement at the former vector interface.
add_line(controller, "ControlInputs/1", "CascadeController/1", ...
    "autorouting", "on");
for destination = ["SelectNextVelocityIntegral", "SelectYawBiasReady", ...
        "SelectMode", "SelectRequestedTorque"]
    add_line(controller, "CascadeController/1", destination + "/1", ...
        "autorouting", "on");
end

% A wider smooth transition makes brief geometric separation/penetration
% less likely to chatter the contact state at the wheel-ball interfaces.
contacts = ["Wheel1BallContact", "Wheel2BallContact", "Wheel3BallContact"];
for k = 1:numel(contacts)
    set_param(targetModel + "/MultibodyPlant/" + contacts(k), ...
        "NormalTransitionRegionWidth", ...
        "ballbotParams.contact.wheelBall.relaxedTransitionWidth");
end

% Idealized contact-retention guide.  It constrains only the distance
% between the rover and ball centers; their relative rotation remains
% free and the three Spatial Contact Force blocks still calculate normal
% and friction forces.  The nominal center distance adds no preload.
ablationModel = "ball_balancing_omni3_ablation_always_on";
open_system(ablationModel);
plant = targetModel + "/MultibodyPlant";
add_block(ablationModel + "/MultibodyPlant/AlwaysOnContactConstraint", ...
    plant + "/ContactRetentionJoint", "Position", [430 790 510 850]);
add_block(ablationModel + "/MultibodyPlant/RoverToBallCenterOffset", ...
    plant + "/ContactRetentionOffset", "Position", [590 790 680 850]);
set_param(plant + "/ContactRetentionOffset", ...
    "TranslationCartesianOffset", ...
    "[0 0 -ballbotParams.rover.centerAboveBall]");
jointPorts = get_param(plant + "/ContactRetentionJoint", "PortHandles");
offsetPorts = get_param(plant + "/ContactRetentionOffset", "PortHandles");
ballPorts = get_param(plant + "/BallSolid", "PortHandles");
roverPorts = get_param(plant + "/ChassisSolid", "PortHandles");
add_line(plant, jointPorts.RConn(1), offsetPorts.RConn(1));
add_line(plant, jointPorts.LConn(1), ballPorts.RConn(1));
add_line(plant, offsetPorts.LConn(1), roverPorts.RConn(1));

set_param(targetModel, "PreLoadFcn", ...
    "ballbotParams = ballbotParameters; ballbotParams.controller.minimumContactConfidence = 0.10;");
ballbotParams = ballbotParameters;
ballbotParams.controller.minimumContactConfidence = 0.10;
assignin("base", "ballbotParams", ballbotParams);
save_system(targetModel);
set_param(targetModel, "SimulationCommand", "update");
save_system(targetModel);
end

function buildCascadeSubsystem(path)
Simulink.SubSystem.deleteContents(path);
add_block("simulink/Sources/In1", path + "/ControlVector", ...
    "Position", [25 210 55 230]);
add_block("simulink/Sinks/Out1", path + "/ControllerOutput", ...
    "Position", [1060 210 1090 230]);

add_block("simulink/Signal Routing/Selector", path + "/Estimate", ...
    "Indices", "1:14", "InputPortWidth", "24", "Position", [90 45 140 75]);
add_block("simulink/Signal Routing/Selector", path + "/Command", ...
    "Indices", "15:17", "InputPortWidth", "24", "Position", [90 95 140 125]);
add_block("simulink/Signal Routing/Selector", path + "/PreviousVelocityIntegral", ...
    "Indices", "18:19", "InputPortWidth", "24", "Position", [90 145 140 175]);
add_block("simulink/Signal Routing/Selector", path + "/Enable", ...
    "Indices", "20", "InputPortWidth", "24", "Position", [90 195 140 225]);
add_block("simulink/Signal Routing/Selector", path + "/PreviousYawReady", ...
    "Indices", "21", "InputPortWidth", "24", "Position", [90 245 140 275]);
add_block("simulink/Signal Routing/Selector", path + "/BiasDiagnostics", ...
    "Indices", "22:24", "InputPortWidth", "24", "Position", [90 295 140 325]);

for name = ["Estimate", "Command", "PreviousVelocityIntegral", ...
        "Enable", "PreviousYawReady", "BiasDiagnostics"]
    add_line(path, "ControlVector/1", name + "/1", "autorouting", "on");
end

add_block("simulink/Signal Routing/Mux", path + "/FrameInputs", ...
    "Inputs", "2", "Position", [180 55 185 115]);
add_block("ball_balancing_omni3_multibody/Controller/ControllerUpdate", ...
    path + "/VelocityErrorBody", ...
    "MATLABFcn", "ballbotVelocityErrorBody(u(1:14),u(15:17))", ...
    "OutputDimensions", "2", ...
    "Position", [220 55 360 115]);
add_line(path, "Estimate/1", "FrameInputs/1", "autorouting", "on");
add_line(path, "Command/1", "FrameInputs/2", "autorouting", "on");
add_line(path, "FrameInputs/1", "VelocityErrorBody/1", "autorouting", "on");

add_block("simulink/Math Operations/Gain", path + "/VelocityP", ...
    "Gain", "diag(ballbotParams.controller.velocityKp)", ...
    "Multiplication", "Matrix(K*u)", "Position", [400 40 475 70]);
add_block("simulink/Math Operations/Gain", path + "/IntegralLeak", ...
    "Gain", "ballbotParams.controller.velocityIntegralLeak", ...
    "Position", [220 145 300 175]);
add_block("simulink/Math Operations/Gain", path + "/IntegralStep", ...
    "Gain", "ballbotParams.controller.sampleTime", ...
    "Position", [400 90 475 120]);
add_block("simulink/Math Operations/Sum", path + "/IntegralUpdate", ...
    "Inputs", "++", "Position", [515 115 535 155]);
add_block("simulink/Discontinuities/Saturation", path + "/IntegralLimit", ...
    "UpperLimit", "ballbotParams.controller.velocityIntegralLimit", ...
    "LowerLimit", "-ballbotParams.controller.velocityIntegralLimit", ...
    "Position", [570 115 650 155]);
add_block("simulink/Math Operations/Gain", path + "/VelocityI", ...
    "Gain", "diag(ballbotParams.controller.velocityKi)", ...
    "Multiplication", "Matrix(K*u)", "Position", [685 115 760 145]);
add_block("simulink/Math Operations/Sum", path + "/SpeedPI", ...
    "Inputs", "++", "Position", [795 55 815 95]);
add_block("simulink/Discontinuities/Saturation", path + "/AccelerationLimit", ...
    "UpperLimit", "ballbotParams.controller.maxPlanarAcceleration", ...
    "LowerLimit", "-ballbotParams.controller.maxPlanarAcceleration", ...
    "Position", [845 55 925 95]);
add_block("simulink/Math Operations/Gain", path + "/AccelerationToLean", ...
    "Gain", "[0 -1/ballbotParams.gravity; 1/ballbotParams.gravity 0]", ...
    "Multiplication", "Matrix(K*u)", "Position", [960 55 1035 95]);

add_line(path, "VelocityErrorBody/1", "VelocityP/1", "autorouting", "on");
add_line(path, "VelocityErrorBody/1", "IntegralStep/1", "autorouting", "on");
add_line(path, "PreviousVelocityIntegral/1", "IntegralLeak/1", "autorouting", "on");
add_line(path, "IntegralLeak/1", "IntegralUpdate/1", "autorouting", "on");
add_line(path, "IntegralStep/1", "IntegralUpdate/2", "autorouting", "on");
add_line(path, "IntegralUpdate/1", "IntegralLimit/1", "autorouting", "on");
add_line(path, "IntegralLimit/1", "VelocityI/1", "autorouting", "on");
add_line(path, "VelocityP/1", "SpeedPI/1", "autorouting", "on");
add_line(path, "VelocityI/1", "SpeedPI/2", "autorouting", "on");
add_line(path, "SpeedPI/1", "AccelerationLimit/1", "autorouting", "on");
add_line(path, "AccelerationLimit/1", "AccelerationToLean/1", "autorouting", "on");

add_block("simulink/Signal Routing/Mux", path + "/BalanceInputs", ...
    "Inputs", "6", "Position", [340 350 345 500]);
add_block("ball_balancing_omni3_multibody/Controller/ControllerUpdate", ...
    path + "/BalanceYawAndAllocation", "Position", [410 350 650 500]);
set_param(path + "/BalanceYawAndAllocation", "MATLABFcn", ...
    "ballbotCascadeBalanceAllocate(u(1:14),u(15:16),u(17:19),u(20),u(21),u(22:24),ballbotParams)");
set_param(path + "/BalanceYawAndAllocation", "OutputDimensions", "5");
add_block("simulink/Signal Routing/Demux", path + "/BalanceOutputDemux", ...
    "Outputs", "[3 1 1]", "Position", [700 350 705 455]);
add_block("simulink/Signal Routing/Mux", path + "/PackOutput", ...
    "Inputs", "4", "Position", [970 195 975 310]);

add_line(path, "Estimate/1", "BalanceInputs/1", "autorouting", "on");
add_line(path, "AccelerationToLean/1", "BalanceInputs/2", "autorouting", "on");
add_line(path, "Command/1", "BalanceInputs/3", "autorouting", "on");
add_line(path, "Enable/1", "BalanceInputs/4", "autorouting", "on");
add_line(path, "PreviousYawReady/1", "BalanceInputs/5", "autorouting", "on");
add_line(path, "BiasDiagnostics/1", "BalanceInputs/6", "autorouting", "on");
add_line(path, "BalanceInputs/1", "BalanceYawAndAllocation/1", "autorouting", "on");
add_line(path, "BalanceYawAndAllocation/1", "BalanceOutputDemux/1", "autorouting", "on");
add_line(path, "BalanceOutputDemux/1", "PackOutput/1", "autorouting", "on");
add_line(path, "IntegralLimit/1", "PackOutput/2", "autorouting", "on");
add_line(path, "BalanceOutputDemux/2", "PackOutput/3", "autorouting", "on");
add_line(path, "BalanceOutputDemux/3", "PackOutput/4", "autorouting", "on");
add_line(path, "PackOutput/1", "ControllerOutput/1", "autorouting", "on");
Simulink.BlockDiagram.arrangeSystem(path);
end
