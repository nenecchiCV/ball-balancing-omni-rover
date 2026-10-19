function [Kv, design] = ballbotVelocityTorqueGains(p, payloadMass, qv)
%BALLBOTVELOCITYTORQUEGAINS LQR velocity columns for the torque-mode LQI.
%   KV = BALLBOTVELOCITYTORQUEGAINS(P, PAYLOADMASS, QV) designs the
%   velocity columns (K10(:, 1:2)) of the torque-mode state gain on a
%   10-state inner plant [vx vy pitch -roll q -p r w1..w3].  The plant
%   extends the 8-state attitude/wheel plant of ballbotRobustTorqueGains
%   with the two planar-velocity states:
%
%     dv/dt = g*[pitch; -roll] + forceScale*W' * tau / mEff
%
%   -- a lean makes the assembly accelerate downhill (the same gravity
%   feed the outer lean loop drives through), and wheel torque produces
%   a direct tangential contact force on the ball (the forceScale=0.5
%   fit measured on open-loop Simscape steps).  The wheel rows keep the
%   -1.35/s compliance drag AND pull toward the kinematic rolling rate
%   W*[v; r] so wheel and velocity states stay consistent.
%
%   With nonzero velocity columns the feedback error (v - v_cmd) maps
%   straight to wheel torque: tracking goes through the contact force,
%   not through a lean setpoint, so braking a drift uses the same
%   symmetric channel as driving.  QV is the planar-velocity weight
%   (diag entry, same units as the attitude weights); the attitude
%   weights and R are shared with ballbotRobustTorqueGains.
%
%   The mass family is solved identically: one dlqr per payload, every
%   candidate cross-applied to every plant, keep the gain whose
%   worst-case closed-loop pole radius over the family is smallest.
%   Only the velocity columns are returned -- the attitude columns of
%   K10 keep the empirical ballbotRobustTorqueGains scalings.

arguments
    p (1, 1) struct
    payloadMass (1, 1) double = 0.5
    qv (1, 1) double = 20
end

measuredB0 = [ ...
    0.0520, -23.5629, 23.5170; ...   % pitch accel  [rad/s^2 per N*m]
    -27.1781, 13.5441, 13.6378; ...   % roll accel
    22.7010, 22.6692, 22.7372; ...    % yaw accel
    305.9042, 32.1404, 32.7251; ...   % wheel1 accel
    32.7227, 305.9042, 32.1425; ...   % wheel2 accel
    32.1404, 32.7227, 305.9652];      % wheel3 accel

payloadGrid = 0:0.25:1.0;
plantCount = numel(payloadGrid);
Ts = p.controller.sampleTime;

[massRef, comRef, tiltRef] = ballbotPayloadRoverProperties(p, 0.5);
IpivotRef = tiltRef + massRef*comRef^2;
IyawRef = p.rover.bodyInertia(3) + 0.5*p.fullplant.payload.width^2/12;

forceScale = 0.5;
if isfield(p, "torqueFit") && isfield(p.torqueFit, "forceScale")
    forceScale = p.torqueFit.forceScale;
end
wheelMap = p.controller.lqi.wheelSpeedFromPlanarVelocity;
ballEffectiveMass = p.ball.mass + p.ball.inertia(1, 1)/p.ball.radius^2;
contactDrag = 1.35;   % wheel-rate pull toward the kinematic rate [1/s]

Q = diag([qv, qv, 200, 200, 20, 20, 5, 0.2, 0.2, 0.2]);
R = 4000*eye(3);

plants = cell(1, plantCount);
gains = cell(1, plantCount);
for i = 1:plantCount
    [mass, comAboveBall, tiltInertiaCom] = ...
        ballbotPayloadRoverProperties(p, payloadGrid(i));
    Ipivot = tiltInertiaCom + mass*comAboveBall^2;
    Iyaw = p.rover.bodyInertia(3) + ...
        payloadGrid(i)*p.fullplant.payload.width^2/12;
    gravityGain = mass*p.gravity*comAboveBall/Ipivot;
    massEffective = mass + ballEffectiveMass;

    measuredB = measuredB0;
    measuredB(1:2, :) = measuredB(1:2, :)*IpivotRef/Ipivot;
    measuredB(3, :) = measuredB(3, :)*IyawRef/Iyaw;

    amp = zeros(10);
    amp(1, 3) = p.gravity;  amp(2, 4) = p.gravity;    % lean -> velocity
    amp(3, 5) = 1;  amp(4, 6) = 1;
    amp(5, 3) = gravityGain;  amp(6, 4) = gravityGain;
    amp(8:10, 8:10) = -contactDrag*eye(3);
    amp(8:10, 1:2) = contactDrag*wheelMap(:, 1:2);    % wheels chase v
    amp(8:10, 7) = contactDrag*wheelMap(:, 3);        % ... and yaw rate
    bmp = zeros(10, 3);
    bmp(1:2, :) = forceScale*wheelMap(:, 1:2).'/massEffective;
    bmp(5, :) = measuredB(1, :);
    bmp(6, :) = -measuredB(2, :);
    bmp(7, :) = measuredB(3, :);
    bmp(8:10, :) = measuredB(4:6, :);
    plants{i} = c2d(ss(amp, bmp, eye(10), zeros(10, 3)), Ts);
    gains{i} = dlqr(plants{i}.A, plants{i}.B, Q, R);
end

closedLoopPoleRadius = zeros(plantCount);
for i = 1:plantCount
    for j = 1:plantCount
        closedLoopPoleRadius(i, j) = max(abs(eig( ...
            plants{j}.A - plants{j}.B*gains{i})));
    end
end
worstPlantRadius = max(closedLoopPoleRadius, [], 2);
[~, chosenDesign] = min(worstPlantRadius);
Kv = gains{chosenDesign}(:, 1:2);

design = struct( ...
    'payloadGrid', payloadGrid, ...
    'closedLoopPoleRadius', closedLoopPoleRadius, ...
    'worstPlantRadius', worstPlantRadius, ...
    'chosenDesign', chosenDesign, ...
    'chosenPayload', payloadGrid(chosenDesign), ...
    'qv', qv, ...
    'velocityGain', Kv, ...
    'fullGain', gains{chosenDesign});
end
