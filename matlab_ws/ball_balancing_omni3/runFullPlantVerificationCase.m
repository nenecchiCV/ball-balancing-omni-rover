function [metrics, out] = runFullPlantVerificationCase(commandXY, stopTime, profileStop)
%RUNFULLPLANTVERIFICATIONCASE Simulate and score one full-plant command.

arguments
    commandXY (2,1) double
    stopTime (1,1) double {mustBePositive} = 4
    profileStop (1,1) double {mustBePositive} = 2.5
end

model = "ball_balancing_omni3_multibody_lqi_custom_contact_fullplant";
p = ballbotFullPlantParameters;
p.command.enable = true;
p.command.profileEnabled = any(commandXY ~= 0);
p.command.profileStart = 0.5;
p.command.profileStop = profileStop;
p.command.profileVelocityWorld = commandXY;
assignin("base", "ballbotParams", p);
in = Simulink.SimulationInput(model);
in = in.setVariable("ballbotParams", p);
in = in.setModelParameter("StopTime", num2str(stopTime), ...
    "MaxStep", num2str(p.simulation.maxStep));
out = sim(in);

time = out.roverPose.Time;
pose = out.roverPose.Data;
velocity = [gradient(pose(:, 1), time), gradient(pose(:, 2), time)];
trackWindow = time >= max(1.5, profileStop - 1) & time < profileStop;
stopWindow = time >= stopTime - 0.5;
metrics.command = commandXY;
metrics.meanTrackingVelocity = mean(velocity(trackWindow, :), 1);
metrics.meanStopVelocity = mean(velocity(stopWindow, :), 1);
metrics.maximumTiltDeg = rad2deg(max(vecnorm(pose(:, 4:5), 2, 2)));
metrics.maximumWheelSpeedCommand = max(abs(out.wheelSpeedCommand.Data), [], "all");
metrics.maximumMotorTorque = max(abs(out.wheelTorqueCommand.Data), [], "all");
metrics.torqueSaturationSeconds = sum(any(abs(out.wheelTorqueCommand.Data) >= ...
    0.999*p.controller.fullplant.motorTorqueLimit, 2))*p.controller.sampleTime;
metrics.minimumNormalForce = [min(out.wheel1NormalForce.Data), ...
    min(out.wheel2NormalForce.Data), min(out.wheel3NormalForce.Data)];
metrics.minimumContactStatus = min(out.contactStatus.Data, [], 1);
metrics.maximumSlip = [maximumVectorNorm(out.wheel1Slip.Data), ...
    maximumVectorNorm(out.wheel2Slip.Data), maximumVectorNorm(out.wheel3Slip.Data)];
metrics.maximumFrictionForce = [maximumVectorNorm(out.wheel1FrictionForce.Data), ...
    maximumVectorNorm(out.wheel2FrictionForce.Data), ...
    maximumVectorNorm(out.wheel3FrictionForce.Data)];
outputNames = who(out);
if any(strcmp(outputNames, "velocityIntegral"))
    metrics.maximumVelocityIntegral = max(abs(out.velocityIntegral.Data), [], 1);
else
    metrics.maximumVelocityIntegral = [NaN, NaN];
end
if any(strcmp(outputNames, "speedIntegral"))
    metrics.maximumSpeedIntegral = max(abs(out.speedIntegral.Data), [], 1);
else
    metrics.maximumSpeedIntegral = [NaN, NaN, NaN];
end
metrics.fallen = metrics.maximumTiltDeg >= rad2deg(p.controller.fallenTilt);
end

function value = maximumVectorNorm(data)
data = squeeze(data);
if size(data, 1) <= 3 && size(data, 2) > size(data, 1)
    data = data.';
end
value = max(vecnorm(data, 2, 2));
end
