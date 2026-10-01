function [K6, info] = ballbotCascadeGains(p, payloadMass)
%BALLBOTCASCADEGAINS Robust LQR for the cascaded tilt+ball-velocity loop.
%   MiaPURE-style architecture (Xiao et al., arXiv:2304.02887): the
%   wheel-torque feedback state includes the BALL/assembly translation
%   velocity, not the wheel rate -- the ball is the load-bearing element
%   and its velocity is the controllable translation coordinate.  State:
%   s = [pitch; -roll; q; -p; vPlanarBody(2)] and the LQR solves
%   u = -K6*(s - s_c) with commanded tilt and commanded velocity
%   supplied by the outer loop.
%
%   The velocity B rows reuse the reduced plant's own convention: the
%   torque->planar-force map is the power dual of the kinematic
%   wheel-rate map, F = W.'*tau (ballbotTorquePlantDynamics), and the
%   assembly acceleration divides by the payload-dependent effective
%   mass.  Tilt rows come from the measured torque-step matrix and
%   scale with the payload pivot inertia.
%
%   Robustness: candidate gains are designed per payload over the mass
%   grid and cross-applied; the gain with the best worst-case closed-
%   loop pole radius is kept -- same procedure as the tilt-only design.

arguments
    p (1, 1) struct
    payloadMass (1, 1) double = 0.5
end

% Measured open-loop B on the nominal 0.5 kg build (torque steps).
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
W = p.controller.lqi.wheelSpeedFromPlanarVelocity;   % 3x3: [vx;vy;yawRate]->w

% Q: tilt tracking dominant, tilt-rate damping moderate, ball-velocity
% tracking modest -- the outer velocity loop owns translation; this
% state mainly damps relative drift, which is what walked the tilt
% into the hold cone in the tilt-only design.
Q = diag([1500, 1500, 150, 150, 30, 30]);
% Control-effort sweep: the saturation cone demands ||K_tilt|| <=
% cap/leanBand, which no single R hits reliably across the payload
% family, so each plant produces a candidate per R value and the
% selector picks the best feasible gain overall.
RGrid = [400, 800, 1600, 3200, 6400, 12000];

% Actuator-lag augmentation: each wheel torque channel gets a lag state
% tau_i, tau_dot = (u_cmd - tau)/tauA.  tauA covers the motor's
% electromechanical constant (30 ms) PLUS the unmodelled loop delays the
% lag must absorb: gyro rate filter (~10 ms equivalent phase) and the
% 5 ms sample ZOH (~2.5 ms) -- 45 ms total keeps the designed phase
% margin honest.  The LQR then returns gains on [x(6); tau(3)] and the
% controller tracks tau internally (it knows the commands it issued),
% feeding back -Klag*tau_est -- the delay-aware equivalent of
% estimating the applied torque.
tauA = 0.045;
if isfield(p.motor, "actuatorLagTimeConstant")
    tauA = max(tauA, 1.5*p.motor.actuatorLagTimeConstant);
end

plants = cell(1, plantCount);      % 6-state physical plants
plantsAug = cell(1, plantCount);   % 9-state lag-augmented plants
candidates = {};                   % each: {gain 3x9, designPayload}
candidateDesign = [];
for i = 1:plantCount
    [mass, comAboveBall, tiltInertiaCom] = ...
        ballbotPayloadRoverProperties(p, payloadGrid(i));
    Ipivot = tiltInertiaCom + mass*comAboveBall^2;
    gravityGain = mass*p.gravity*comAboveBall/Ipivot;
    massEffective = mass + p.ball.mass + ...
        p.ball.inertia(1, 1)/p.ball.radius^2;
    tiltRows = measuredB0(1:2, :)*IpivotRef/Ipivot;
    % Empirical coupling: the compliant wheel-ball contact delivers only
    % ~forceScale of the rigid power-dual force (fitted from open-loop
    % torque steps on the Simscape plant, see p.torqueFit).
    forceScale = 1;
    if isfield(p, "torqueFit") && isfield(p.torqueFit, "forceScale")
        forceScale = p.torqueFit.forceScale;
    end
    Bvel = forceScale*W(:, 1:2).'/massEffective;    % 2x3, m/s^2 per N*m

    amp = zeros(6);
    amp(1, 3) = 1; amp(2, 4) = 1;
    amp(3, 1) = gravityGain; amp(4, 2) = gravityGain;
    bmp = zeros(6, 3);
    bmp(3:4, :) = [tiltRows(1, :); -tiltRows(2, :)];
    bmp(5:6, :) = Bvel;
    plants{i} = c2d(ss(amp, bmp, eye(6), zeros(6, 3)), Ts);
    if tauA > 0
        % Continuous lag augmentation then discretise.
        Aa = [amp, bmp; zeros(3, 6), -eye(3)/tauA];
        Ba = [zeros(6, 3); eye(3)/tauA];
        sysA = c2d(ss(Aa, Ba, eye(9), zeros(9, 3)), Ts);
        plantsAug{i} = sysA;
        Qa = blkdiag(Q, zeros(3));
        for rScale = RGrid
            candidates{end + 1} = dlqr(sysA.A, sysA.B, Qa, ...
                rScale*eye(3)); %#ok<AGROW>
            candidateDesign(end + 1) = payloadGrid(i); %#ok<AGROW>
        end
    else
        plantsAug{i} = [];
        for rScale = RGrid
            candidates{end + 1} = [dlqr(plants{i}.A, plants{i}.B, ...
                Q, rScale*eye(3)), zeros(3)]; %#ok<AGROW>
            candidateDesign(end + 1) = payloadGrid(i); %#ok<AGROW>
        end
    end
end
gains = candidates;
designCount = numel(gains);

closedLoopPoleRadius = zeros(designCount, plantCount);
for i = 1:designCount
    for j = 1:plantCount
        % Closed-loop poles on the full lag-augmented plant: the lag
        % states feed back through Klag, so the Kx-only subsystem is not
        % a valid stability check.
        if ~isempty(plantsAug{j})
            closedLoopPoleRadius(i, j) = max(abs(eig( ...
                plantsAug{j}.A - plantsAug{j}.B*gains{i})));
        else
            closedLoopPoleRadius(i, j) = max(abs(eig( ...
                plants{j}.A - plants{j}.B*gains{i}(:, 1:6))));
        end
    end
end
worstPlantRadius = max(closedLoopPoleRadius, [], 2);
% Static-hold constraint: the tilt stiffness of the chosen gain must
% clear the heaviest plant's gravity floor B*Kt > gg with margin, or
% the startup transient sits above the hold cone and no feedback can
% catch it.  The pure pole-radius criterion ignores this because it
% only sees asymptotic stability; a marginally-stable light-design
% gain (Kt ~ 2.9 < 4.17 floor) tipped the 1 kg build at t ~0.8 s.
% Static-hold constraint: the feedback tilt stiffness B_tilt*K_tilt must
% exceed the gravity anti-stiffness gg with margin on EVERY family
% plant, else the startup transient sits above the hold cone and no
% feedback can catch it (a marginally-stable 0.5 kg design with
% Kt ~ 2.9 tipped the 1 kg build at t ~0.8 s).  The exact condition is
% eig(Btilt_j * K(1:2 cols)) > gg_j, not a singular-value bound.
tiltB = cell(1, plantCount);
gravityGainGrid = zeros(1, plantCount);
for i = 1:plantCount
    [mass, comAboveBall, tiltInertiaCom] = ...
        ballbotPayloadRoverProperties(p, payloadGrid(i));
    Ipivot = tiltInertiaCom + mass*comAboveBall^2;
    gravityGainGrid(i) = mass*p.gravity*comAboveBall/Ipivot;
    tiltRows = measuredB0(1:2, :)*IpivotRef/Ipivot;
    tiltB{i} = [tiltRows(1, :); -tiltRows(2, :)];
end
holdMargin = 1.3;
% Saturation cone constraint: the tilt feedback stays linear only while
% K*theta < cap; beyond leanBand the demand clips to the traction budget
% and the loop degenerates to bang-bang (measured: demand at 0.05 rad
% was ~0.5 N*m vs a 0.04 N*m cap -- the loop had no linear range at
% all).  Feasibility is checked on the TARGET payload only: floor and
% cone do not intersect for a single gain across the whole family
% (light builds cap out before the heavy build's floor clears), so the
% gain is payload-scheduled -- the physical payload is bolted and known.
% tractionLimitedWheelTorque already scales with this payload's normal
% load (see ballbotRev1Parameters), so the target-plant cap is the field
% itself -- no further mass scaling.
targetIndex = find(payloadGrid >= payloadMass, 1);
if isempty(targetIndex), targetIndex = plantCount; end
capTarget = p.controller.tractionLimitedWheelTorque;
leanBand = 0.015;
tiltColumn = zeros(1, designCount);   % largest of the two tilt cols
feasible = false(1, designCount);
for i = 1:designCount
    tiltColumn(i) = max([norm(gains{i}(:, 1)), norm(gains{i}(:, 2))]);
    stiffness = eig(tiltB{targetIndex}*gains{i}(:, 1:2));
    feasible(i) = all(isreal(stiffness)) && ...
        min(real(stiffness)) > holdMargin*gravityGainGrid(targetIndex) && ...
        tiltColumn(i) <= capTarget/leanBand;
end
% Among feasible designs pick the fastest closed-loop radius on the
% TARGET plant (the gain is payload-scheduled, so target-plant damping
% is the objective; family radius is kept in info as a robustness
% diagnostic).  When nothing clears floor+cone on the target, fall back
% to the largest tilt column (maximal capture authority) as before.
score = closedLoopPoleRadius(:, targetIndex);
if any(feasible)
    score(~feasible) = inf;
    [~, chosenDesign] = min(score);
else
    [~, chosenDesign] = max(tiltColumn);
end
K9 = gains{chosenDesign};
K6 = K9(:, 1:6);
Klag = K9(:, 7:9);

info = struct( ...
    'payloadGrid', payloadGrid, ...
    'closedLoopPoleRadius', closedLoopPoleRadius, ...
    'worstPlantRadius', worstPlantRadius, ...
    'chosenDesign', chosenDesign, ...
    'chosenPayload', candidateDesign(chosenDesign), ...
    'gravityGainGrid', gravityGainGrid, ...
    'holdFeasible', feasible, ...
    'tiltColumnNorm', tiltColumn, ...
    'leanBand', leanBand, ...
    'appliedTorqueGain', Klag, ...
    'actuatorLagTimeConstant', tauA, ...
    'BvelNominal', forceScale*W(:, 1:2).'/(p.rover.mass + p.ball.mass + ...
        p.ball.inertia(1, 1)/p.ball.radius^2));
end
