% Run the 3WD omnirover world-frame command demonstration.
run("parameters.m");

in = Simulink.SimulationInput("omnirover3wd_multibody");
in = in.setModelParameter("StopTime", num2str(simulationStopTime));
out = sim(in);

pose = out.roverPose;
figure(Name="3WD Omnirover World-Frame Response");
tiledlayout(2,1);
nexttile;
plot(pose.Time, pose.Data(:,1:2), LineWidth=1.5);
grid on;
ylabel("Position (m)");
legend("x_W", "y_W", Location="best");
nexttile;
plot(pose.Time, rad2deg(pose.Data(:,3)), LineWidth=1.5);
grid on;
xlabel("Time (s)");
ylabel("Yaw (deg)");

fprintf("Final pose: x=%.4f m, y=%.4f m, yaw=%.3f deg\n", ...
    pose.Data(end,1), pose.Data(end,2), rad2deg(pose.Data(end,3)));

