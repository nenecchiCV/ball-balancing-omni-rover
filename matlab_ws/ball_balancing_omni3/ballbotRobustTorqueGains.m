function [K10, robustness] = ballbotRobustTorqueGains(p)
%BALLBOTROBUSTTORQUEGAINS Fixed torque-mode state gain over the mass family.
%   [K10, ROBUSTNESS] = BALLBOTROBUSTTORQUEGAINS(P) designs one fixed
%   wheel-torque state gain for the whole payload range instead of the
%   nominal mass.  The open-loop measured B matrix (0.01 N*m torque steps
%   on the assembled rev1 plant) is taken at the nominal 0.5 kg build;
%   for each candidate payload the tilt rows scale with 1/Ipivot, the yaw
%   row with 1/Iyaw, the wheel rows are payload-independent (fixed rotor
%   inertia), and the unstable tilt coefficient becomes m*g*h/Ipivot.
%
%   A discrete LQR is solved per mass on the 8-state inner plant
%   [pitch, -roll, q, -p, r, w1..w3], every candidate gain is then applied
%   to every plant, and the gain with the best worst-case closed-loop pole
%   radius is kept.  Designing at the light end fails on heavy plants
%   (their per-torque tilt authority is ~8x lower), so the winner is always
%   a heavy-end design; the torque clamp then bounds the aggressive
%   response on light builds, whose authority margin covers saturation.
%
%   ROBUSTNESS fields: payloadGrid, closedLoopPoleRadius (design x plant
%   matrix), worstPlantRadius per candidate, chosenDesign index, K10 is the
%   3x10 state-gain matrix with the empirical column scalings already
%   applied (tilt x4, tilt-rate x0.125, yaw-rate x5 -- the measured
%   corrections for stiction, electrical lag and yaw authority; the tilt
%   pair was halved after the settled-contact fix, see below).

arguments
    p (1, 1) struct
end

% Open-loop torque-step B measured on the nominal (0.5 kg payload) build.
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

% Reference mass properties at the nominal payload where B was measured.
[massRef, comRef, tiltRef] = ballbotPayloadRoverProperties(p, 0.5);
IpivotRef = tiltRef + massRef*comRef^2;
IyawRef = p.rover.bodyInertia(3) + 0.5*p.fullplant.payload.width^2/12;

Q = diag([200, 200, 20, 20, 5, 0.2, 0.2, 0.2]);
R = 2000*eye(3);

plants = cell(1, plantCount);
gains = cell(1, plantCount);
for i = 1:plantCount
    [mass, comAboveBall, tiltInertiaCom] = ...
        ballbotPayloadRoverProperties(p, payloadGrid(i));
    Ipivot = tiltInertiaCom + mass*comAboveBall^2;
    Iyaw = p.rover.bodyInertia(3) + ...
        payloadGrid(i)*p.fullplant.payload.width^2/12;
    gravityGain = mass*p.gravity*comAboveBall/Ipivot;

    measuredB = measuredB0;
    measuredB(1:2, :) = measuredB(1:2, :)*IpivotRef/Ipivot;
    measuredB(3, :) = measuredB(3, :)*IyawRef/Iyaw;

    amp = zeros(8);
    amp(1, 3) = 1; amp(2, 4) = 1;
    amp(3, 1) = gravityGain; amp(4, 2) = gravityGain;
    amp(6, 6) = -1.35; amp(7, 7) = -1.35; amp(8, 8) = -1.35;
    bmp = zeros(8, 3);
    bmp(3, :) = measuredB(1, :);
    bmp(4, :) = -measuredB(2, :);
    bmp(5, :) = measuredB(3, :);
    bmp(6:8, :) = measuredB(4:6, :);
    plants{i} = c2d(ss(amp, bmp, eye(8), zeros(8, 3)), Ts);
    gains{i} = dlqr(plants{i}.A, plants{i}.B, Q, R);
end

% Cross-apply every candidate gain to every plant; keep the candidate
% whose worst-case closed-loop pole radius over the family is smallest.
closedLoopPoleRadius = zeros(plantCount);
for i = 1:plantCount
    for j = 1:plantCount
        closedLoopPoleRadius(i, j) = max(abs(eig( ...
            plants{j}.A - plants{j}.B*gains{i})));
    end
end
worstPlantRadius = max(closedLoopPoleRadius, [], 2);
[~, chosenDesign] = min(worstPlantRadius);
KMeasured = gains{chosenDesign};

K10 = zeros(3, 10);
% Column scalings x4 / x0.125: the earlier x8 / x0.25 pair was tuned while
% the initial-height bug had the rover bouncing on the ball -- the violent
% impacts needed an aggressively saturated tilt channel.  With the contact
% now settled continuously (4.07 N steady), the same gains saturate the
% traction limit into a bang-bang limit cycle (~tilt 0.002 rad already
% commands the clamp) and halving both tilt channels roughly tripled the
% measured tip-over time (1.1 s -> 3.0 s cycle runs).
K10(:, 3:4) = 4*KMeasured(:, 1:2);      % tilt [pitch, -roll]
K10(:, 5:6) = 0.125*KMeasured(:, 3:4);  % tilt rate [q, -p]
% Yaw-rate column restored but gated: the A/B experiment (estimator vs
% plant-truth feedback, identical otherwise) showed continuous yaw-rate
% feedback is the dominant destabilizer -- common-mode wheel torque
% shuffles angular momentum between chassis yaw and the ball's free
% spin mode until the contacts unload (~5 s tip-over).  With the channel
% gated to near-rest windows only (yawGate in ballbotRev1Parameters), the
% pump injects a bounded amount of ball spin per manoeuvre instead of
% accumulating it.
K10(:, 7) = 5*KMeasured(:, 5);         % yaw rate (stationary gate)

robustness = struct( ...
    'payloadGrid', payloadGrid, ...
    'closedLoopPoleRadius', closedLoopPoleRadius, ...
    'worstPlantRadius', worstPlantRadius, ...
    'chosenDesign', chosenDesign, ...
    'chosenPayload', payloadGrid(chosenDesign));
end
