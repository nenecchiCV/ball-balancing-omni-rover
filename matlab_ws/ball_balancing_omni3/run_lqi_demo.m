% Run the LQI ball-balancing 3WD omnirover demonstration.
p = ballbotParameters;
p.command.velocityWorld = [0; 0];
p.controller.minimumContactConfidence = 0.10;
assignin("base", "ballbotParams", p);

modelName = "ball_balancing_omni3_multibody_lqi";
in = Simulink.SimulationInput(modelName);
in = in.setModelParameter("StopTime", num2str(p.simulation.stopTime));
out = sim(in);

roverPose = out.roverPose;
ballPose = out.ballPose;
figure(Name="Ball-Balancing 3WD Omnirover LQI Response");
tiledlayout(3, 1);
nexttile;
plot(roverPose.Time, roverPose.Data(:, 1:2), LineWidth=1.5);
grid on;
ylabel("Rover position (m)");
legend("x_W", "y_W", Location="best");
nexttile;
plot(roverPose.Time, rad2deg(roverPose.Data(:, 4:6)), LineWidth=1.5);
grid on;
ylabel("Rover attitude (deg)");
legend("roll", "pitch", "yaw", Location="best");
nexttile;
plot(ballPose.Time, ballPose.Data(:, 1:2), LineWidth=1.5);
grid on;
xlabel("Time (s)");
ylabel("Ball center (m)");
legend("x_W", "y_W", Location="best");
