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
R = 800*eye(3);

plants = cell(1, plantCount);
gains = cell(1, plantCount);
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
feasible = false(1, plantCount);
tiltColumn = zeros(1, plantCount);
for i = 1:plantCount
    tiltColumn(i) = min([norm(gains{i}(:, 1)), norm(gains{i}(:, 2))]);
    ok = true;
    for j = 1:plantCount
        stiffness = eig(tiltB{j}*gains{i}(:, 1:2));
        if any(~isreal(stiffness)) || ...
                min(real(stiffness)) <= holdMargin*gravityGainGrid(j)
            ok = false;
            break
        end
    end
    feasible(i) = ok;
end
% Among feasible designs pick the largest tilt column: the recoverable
% cone is theta_cone = svd_min(Btilt)*tau_limit/gg, which narrows toward
% the heavy end (~0.015 rad at 1 kg -- the size of the startup
% transient), so the heaviest plant needs the most torque per radian to
% catch the transient before it leaves the cone.  Pole radius is nearly
% flat across the family (0.9955-0.998) and is not the discriminator.
if any(feasible)
    tiltColumn(~feasible) = -inf;
end
[~, chosenDesign] = max(tiltColumn);
K6 = gains{chosenDesign};

info = struct( ...
    'payloadGrid', payloadGrid, ...
    'closedLoopPoleRadius', closedLoopPoleRadius, ...
    'worstPlantRadius', worstPlantRadius, ...
    'chosenDesign', chosenDesign, ...
    'chosenPayload', payloadGrid(chosenDesign), ...
    'gravityGainGrid', gravityGainGrid, ...
    'holdFeasible', feasible, ...
    'tiltColumnNorm', tiltColumn, ...
    'BvelNominal', forceScale*W(:, 1:2).'/(p.rover.mass + p.ball.mass + ...
        p.ball.inertia(1, 1)/p.ball.radius^2));
end
