function p = ballbotParameters(sampleTime, design)
%BALLBOTPARAMETERS Nominal plant, estimator, and controller parameters.
%   p = BALLBOTPARAMETERS uses the 5 ms (200 Hz) estimator/controller
%   discrete rate the NUCLEO deployment was sized for.
%   p = BALLBOTPARAMETERS(sampleTime) overrides that rate [s]; every
%   Ts-derived quantity (encoder flicker thresholds, filter weights,
%   discrete gain redesign) is built from it consistently.
%   p = BALLBOTPARAMETERS(sampleTime, design) additionally applies
%   DESIGN, a cell array of {dottedPath, value} pairs (same convention
%   as ballbotBenchmark overrides), BEFORE the geometry/gain
%   derivations so
%   chassis-design knobs (wheel.contactLatitude, ball.radius, ...)
%   propagate consistently.  Late overrides still clobber the values
%   their dependents were built from -- use this path, not post-hoc
%   p-field edits, for geometry what-ifs.

if nargin < 1 || isempty(sampleTime)
    sampleTime = 0.005;
end

p.gravity = 9.80665;

% Reference ball: 100 mm, 0.285 kg rigid ball with thin-shell inertia.
p.ball.radius = 0.050;
p.ball.mass = 0.285;
p.ball.inertia = (2/3)*p.ball.mass*p.ball.radius^2*eye(3);

% goBILDA/ServoCity 48 mm omniwheel (3624-4008-0048, 8 mm REX bore):
% two staggered rows of six 50A-durometer rubber rollers; the second row
% bridges the roller gaps, so contact is effectively continuous around
% the circumference (vs the previous single-row Nexus 14108).  Overall
% width 18 mm from the manufacturer's schematic.
p.wheel.radius = 0.024;
p.wheel.width = 0.018;
p.wheel.mass = 0.032;
p.wheel.rollerCount = 12;
p.wheel.azimuth = deg2rad([0; 120; 240]);
% Wheel/ball contact latitude (elevation of the wheel normal above the
% ball's equatorial plane; zenith angle = 90-latitude).  Conventional
% ballbots use latitude 40-55 deg.  The previous 65 deg ("coaxial
% outward pod maximum") starved every wheel of normal load
% (N = mg/(3 sin lambda) grows 41% at 40 deg): the deceleration-corner
% traction wall, the 8.6 s yaw fall, and the ~0.87 deg hold jitter all
% traced back to that shortfall.  Measured at 5 ms / torque-LQI /
% realistic plant (cycle move+stop, hold, stepX, yaw, bias, CG-20 mm):
%   65deg: cyc 43.8 s fall / hold 0.87deg / yaw 8.6 s fall
%   55deg: cyc 26.8 s      60deg: cyc 41.7 s
%   50deg: cyc completes only with eccentricity guard OFF
%   45deg: cyc completes 0.62deg 6/9
%   40deg: cyc completes 0.27deg 7/9, hold 0.19deg, yaw completes,
%          CG-20 mm holds 2.06 deg
%   33deg: cyc completes 0.35deg 7/9, hold 0.15deg, yaw completes,
%          CG-20 mm holds 1.62 deg (inward-pod mass model) -- adopted
%          to enable plan-B inward motor storage: inward pods only fit
%          for lambda <~ 33 deg (three ~67 mm motors converging on the
%          axis need tail radius >= ~16 mm), and 33 deg keeps
%          control metrics equal-or-better than 40 deg while gaining
%          +18% wheel normal load.  Outward pods are impossible here --
%          the motor tail would reach the floor (z ~ 5 mm).
p.wheel.contactLatitude = deg2rad(33);
% Small geometric preload keeps all three nominally tangent contacts
% active despite contact-detection and floating-point tolerances.
p.wheel.contactPreload = 5.0e-5;

% DFRobot FIT0521 6 V geared DC motor with quadrature Hall encoder.
p.motor.dimensions = [0.0520, 0.0244, 0.0244];
p.motor.mass = 0.096;
p.motor.nominalVoltage = 6.0;
p.motor.gearRatio = 34.02;
p.motor.noLoadSpeed = 210*2*pi/60;
p.motor.noLoadCurrent = 0.13;
p.motor.stallTorque = 10*9.80665/100;
p.motor.stallCurrent = 3.2;
p.motor.armatureResistance = ...
    p.motor.nominalVoltage/p.motor.stallCurrent;
p.motor.torqueConstant = p.motor.stallTorque/p.motor.stallCurrent;
p.motor.backEmfConstant = ...
    (p.motor.nominalVoltage - ...
    p.motor.noLoadCurrent*p.motor.armatureResistance)/ ...
    p.motor.noLoadSpeed;
p.motor.viscousFriction = ...
    p.motor.torqueConstant*p.motor.noLoadCurrent/p.motor.noLoadSpeed;
p.motor.timeConstant = 0.030;
p.motor.equivalentInertia = p.motor.timeConstant* ...
    p.motor.stallTorque/p.motor.noLoadSpeed;

% Onboard power source: Zeee 7.2 V 5000 mAh NiMH pack (6 sub-C cells,
% TAMIYA plug), mounted at the centre of the first chassis layer.  Fully
% charged ~8.4-8.7 V, discharged ~6.0 V; parameters use the nominal
% point.  ~90 mOhm pack resistance sags ~0.3 V at 3 A; the H-bridge
% supply is ideal, so sag is folded into the duty cap below.
p.battery.nominalVoltage = 7.2;
p.battery.chargedVoltage = 8.4;
p.battery.cutoffVoltage = 6.0;
p.battery.capacityAh = 5.0;
p.battery.internalResistance = 0.09;
p.battery.mass = 0.423;
p.battery.dimensions = [0.140, 0.050, 0.025];

% Two Cytron MDD3A boards provide the three required PWM/DIR channels.
p.driver.boardCount = 2;
p.driver.channelCountPerBoard = 2;
p.driver.supplyVoltage = p.battery.nominalVoltage;
p.driver.continuousCurrent = 3.0;
p.driver.peakCurrent = 5.0;
p.driver.pwmFrequency = 20e3;
% The pack's nominal 7.2 V exceeds the FIT0521's 6 V rating, so cap the
% duty where the average winding voltage equals the rating (plus the
% ~0.3 V worst-case pack sag already buys a little margin).  The motor
% constants are still the 6 V datasheet measurements, so PWM peaks at
% 7.2 V while the average stays inside spec.
p.driver.maxDutyCycle = min(0.95, ...
    p.motor.nominalVoltage/p.battery.nominalVoltage);
p.driver.continuousTorqueLimit = p.motor.torqueConstant* ...
    min(p.driver.continuousCurrent, p.motor.stallCurrent);
p.driver.peakTorqueLimit = p.motor.stallTorque;

p.encoder.supplyVoltage = 3.3;
p.encoder.pulsesPerOutputRevolution = 341.2;
p.encoder.quantization = 2*pi/p.encoder.pulsesPerOutputRevolution;
% Datasheet wording ("11 counts/rev motor shaft -> 341.2 PPR output
% shaft") is ambiguous about whether the 11 are quadrature counts or
% single-channel cycles.  If the firmware decoder counts x4 edges and
% the PPR is per-cycle, the effective quantum is 4x smaller; left at 1
% until bench-verified (conservative = more flicker modeled).
p.encoder.quadratureFactor = 1;
% Wheel-rate measurement model: per-sample backward difference of the
% quantized displacement (flicker quant/Ts).

% Output-shaft-equivalent speed loop.  The 30 ms time constant is
% provisional until a FIT0521 step response is measured under load.
p.motor.speedControllerNaturalFrequency = 30;
p.motor.speedControllerDamping = 0.90;
p.motor.speedPlantDcGain = ...
    p.motor.noLoadSpeed/p.driver.supplyVoltage;
p.motor.speedControllerKp = ...
    (2*p.motor.speedControllerDamping* ...
    p.motor.speedControllerNaturalFrequency*p.motor.timeConstant - 1)/ ...
    p.motor.speedPlantDcGain;
p.motor.speedControllerKi = ...
    p.motor.timeConstant*p.motor.speedControllerNaturalFrequency^2/ ...
    p.motor.speedPlantDcGain;
p.motor.speedControllerAntiWindup = 1/max(p.motor.speedControllerKp, eps);
p.motor.speedControllerIntegralLimit = ...
    p.driver.supplyVoltage/p.motor.speedControllerKi;
p.motor.speedCommandLimit = ...
    p.driver.maxDutyCycle*p.motor.noLoadSpeed;
p.motor.torquePerSpeedError = ...
    (p.driver.continuousTorqueLimit/p.driver.supplyVoltage)* ...
    p.motor.speedControllerKp;

% Legacy aliases keep archived model variants loadable during migration.
p.servo = p.motor;
p.servo.maxTorque = p.motor.stallTorque;
p.servo.maxSpeed = p.motor.noLoadSpeed;

% Redesigned triangular body. The body frame origin is the IMU location.
% Chassis lump follows the ball_balancing_omnirover3wd_assembly frame
% structure: three ABS frame plates (D160x3 at z = 148.5/201.5/254.5 mm
% above ground), twelve M3x50 ABS spacers (OD 6.2 / bore 3.2 mm) at
% r = 65 mm on the 30-deg-rotated pattern {0,+-60,+-120,180}, three ABS
% MotorMount_v23 inward-pod square-tube mounts below L1 (single-body
% 48x48 mm square tube + face plate + frame-side wall extended to a
% uniform rectangular fastening plate x=34..72.5 x y=+-34, wedge
% fills so the fastening-plate face is the print-bed face, motor-end
% bottom corner chamfered on the face-plate plane (parallel to the
% ball-wheel contact tangent, +1.358 mm deeper so the chamfer merges
% with the plate face); wheel
% adapters included), and the L1/L2
% electronics payload (Zeee 7.2 V NiMH pack centred on L1, two MDD3A
% drivers, Nucleo+IKS4A1 stack).
% Heights below are measured above the ball center (ground z = 50 mm).
% lambda = 33 deg PLAN-B INWARD pod geometry (57 deg vehicle-axis tilt,
% motors stowed inward-up toward the axis through L1 clearance holes).
% Fusion-measured values from the lambda33 assembly (MotorMount_v23):
% single-body square tube wrapping the motor can, face plate bolted
% to the PCD-17 pattern via M2.5 screws, fastening plate bolted to L1
% via M3 bolt+nut+washer sandwiches at (44.5,+-29) and (62,+-29) mm;
% the clamp strap was deleted (no room
% behind the wheel hub bracket).  Wheel
% arm/height rows are exact from geometry.
rhoABS = 1050;
l1DiscMass = rhoABS*43.994e-6;   % L1 disc net of openings + 30 x Ø3.5 M3-clearance holes incl. rotated spacer pattern (Fusion measured 43.994 cm^3)
l1DiscInertia = l1DiscMass*(3*0.080^2 + 0.003^2)/12;
frameDiscMass = rhoABS*59.203e-6; % L2 disc net of 6 x Ø3.5 spacer holes + Ø20 center hole (Fusion measured 59.203 cm^3; obsolete spacer-hole fill stubs removed)
frameDiscInertia = frameDiscMass*(3*0.080^2 + 0.003^2)/12;
l3DiscMass = rhoABS*60.146e-6;    % L3 disc: same as L2 but Ø20 center hole filled (unneeded); Fusion measured 60.146 cm^3
l3DiscInertia = l3DiscMass*(3*0.080^2 + 0.003^2)/12;
spacerMass = rhoABS*1.1074e-6;   % Spacer_M3x50 tube OD 6.2 / bore 3.2 x 50 mm (Fusion measured 1.1074 cm^3)
spacerInertia = spacerMass*(3*(0.0031^2+0.0016^2) + 0.050^2)/12;
mountMass = rhoABS*51.443e-6;    % MotorMount_v23 plate-width square-tube mount + rectangular fastening plate + wedge fills + motor-end bottom-corner chamfer on the face-plate plane (parallel to the ball-wheel contact tangent; M2.5 face holes + central Ø25 boss; single body, 60 faces), measured 51.443 cm^3
adapterMass = rhoABS*1.2171e-6;  % WheelAdapter_v4 measured volume 1.217 cm^3 (tube s'=-1.0..11.5)
% Fusion-measured own-CG inertias scaled to ABS mass (the y/tangential
% axis entry is used for tilt; the z axis entry is used in the yaw sum).
% v23 CG-frame tensor (steel) Ixx/Iyy/Izz = 2.012/1.181/1.761
% kg*cm^2 (Ixz = -0.12) -> ABS scale 0.1338.
mountTiltI = 1.58e-5;  % per MotorMount_v23 (own-CG, ABS-scaled)
mountYawI = 2.36e-5;
adapterTiltI = adapterMass*(3*0.006^2 + 0.0125^2)/12;
adapterYawI = adapterMass*0.006^2/2;
% Drivetrain CG heights [m above ball center] and radial arms [m] for
% the lambda33 inward pod.  Wheel values are exact geometry
% (74*sin(33deg) / 74*cos(33deg)); motor/mount values are Fusion CG
% measurements of the placed occurrences.
motorComAboveBall = 0.0790;   % Fusion motor CG z = 129.0 mm above ground
wheelComAboveBall = 0.0403;
mountComAboveBall = 0.0783;   % Fusion v23 CG z = 128.3 mm above ground (78.3 above ball centre z=50)
adapterComAboveBall = 0.0447;   % Fusion adapter CG z = 94.7 mm above ground
motorYawArm = 0.0371;         % Fusion motor CG radial arm
wheelYawArm = 0.0620;
mountYawArm = 0.0556;         % Fusion v23 CG radial arm
adapterYawArm = 0.0592;       % Fusion adapter CG radial arm
motorTiltI = p.motor.mass*(p.motor.dimensions(1)^2 + ...
    p.motor.dimensions(2)^2)/12;
wheelTiltI = p.wheel.mass*(3*p.wheel.radius^2 + ...
    p.wheel.width^2)/12;
chassisParts = [ ...
%    mass[kg]        comZ above ball center [m]  Ixx about own COM [kg m^2]
    l1DiscMass      0.0985                      l1DiscInertia
    frameDiscMass   0.1515                      frameDiscInertia
    l3DiscMass      0.2045                      l3DiscInertia
    6*spacerMass    0.12525                     6*spacerInertia
    6*spacerMass    0.17825                     6*spacerInertia
    3*mountMass     mountComAboveBall           3*mountTiltI
    3*adapterMass   adapterComAboveBall         3*adapterTiltI
    0.423           0.1125                      0.423*(0.140^2+0.025^2)/12
    0.030           0.103                       0.030*(0.050^2+0.030^2)/12
    0.090           0.156                       0.090*(0.133^2+0.080^2)/12
    0.010           0.110                       0];
p.rover.bodyMass = sum(chassisParts(:, 1));
p.rover.bodyComAboveBall = ...
    sum(chassisParts(:, 1).*chassisParts(:, 2))/p.rover.bodyMass;
offAxisYaw = 6*spacerMass*0.065^2*2 + 3*mountMass*mountYawArm^2 + ...
    3*adapterMass*adapterYawArm^2 + 3*p.motor.mass*motorYawArm^2 + ...
    3*p.wheel.mass*wheelYawArm^2;
driveTiltInertia = 3*(motorTiltI + p.motor.mass* ...
    (motorComAboveBall - p.rover.bodyComAboveBall)^2) + ...
    3*(wheelTiltI + p.wheel.mass* ...
    (wheelComAboveBall - p.rover.bodyComAboveBall)^2);
bodyTiltInertia = sum(chassisParts(:, 3) + chassisParts(:, 1).* ...
    (chassisParts(:, 2) - p.rover.bodyComAboveBall).^2) + offAxisYaw/2 ...
    + driveTiltInertia;
% Own-CG yaw inertias: disc/rod rows keep the Izz ~= 2*Ixx
% approximation; the pod rows use the Fusion-measured yaw values.
ownYaw = 2*chassisParts(:, 3);
ownYaw(6) = 3*mountYawI;
ownYaw(7) = 3*adapterYawI;
p.rover.bodyInertia = [bodyTiltInertia; bodyTiltInertia; ...
    sum(ownYaw) + offAxisYaw];
p.rover.mass = p.rover.bodyMass + 3*(p.motor.mass + p.wheel.mass);
p.rover.centerAboveBall = (p.rover.bodyComAboveBall*p.rover.bodyMass + ...
    3*(p.motor.mass*motorComAboveBall + ...
    p.wheel.mass*wheelComAboveBall))/p.rover.mass;
p.rover.chassisRadius = 0.080;
p.rover.chassisHeight = 0.030;
p.rover.initialPositionWorld = [0; 0; ...
    p.ball.radius + p.rover.centerAboveBall];
p.rover.initialEulerWorld = deg2rad([0; 0; 0]);
p.rover.noseAxisBody = [1; 0; 0];

% Penalty contacts, idealized environment: near-critically damped normals
% kill the settle bounce (bare values gave zeta ~0.1-0.4, the seed of the
% startup tipping mode), and high fixed friction keeps wheel slip
% controller-limited instead of contact-limited.  The wheel-slip
% acceleration limit the controller must respect is NOT modeled here --
% it lives in tractionLimitedWheelTorque below, which keeps the
% effective mu=0.80 budget even though the contact itself cannot slip.
% Ball compliance model: hollow Ø100 plastic shell + ~1-2 mm rubber
% membrane -- the membrane, not the shell, sets the contact stiffness.
% Wheel side adds the goBILDA roller in SERIES: 50A-durometer rubber
% sleeve ~1.5 mm thick over the plastic roller core (E ~2.5 MPa,
% contact patch ~1.5e-5 m^2 -> ~2.5e4 N/m), so the wheel-ball
% interface is the series combination ~1.4e4 N/m, NOT the ball
% membrane alone.  Sag check at ~8.5 N nominal wheel load: ~0.6 mm
% total (roller + membrane), consistent with each layer compressing
% a fraction of its thickness.  The floor contact below has no
% roller layer and keeps the membrane-dominated value.  All values
% remain bench-measure candidates before hardware sign-off.
p.contact.wheelBall.ballMembraneStiffness = 3.0e4;
p.contact.wheelBall.rollerLayerStiffness = 2.5e4;
p.contact.wheelBall.normalStiffness = 1/(1/3.0e4 + 1/2.5e4);   % ~1.4e4
p.contact.wheelBall.normalDamping = 170;   % ~critical for k=1.4e4, m~0.5 kg/wheel
p.contact.wheelBall.transitionWidth = 5.0e-4;
% Wheel-ball contact transition width wired into the WheelNBallContact
% blocks; the relaxed value damps contact-state chatter.
p.contact.wheelBall.relaxedTransitionWidth = 1.5e-3;
p.contact.wheelBall.staticFriction = 2.0;
p.contact.wheelBall.dynamicFriction = 2.0;
p.contact.wheelBall.rollerFriction = 0.02;
p.contact.wheelBall.criticalVelocity = 5.0e-3;
% 1e4 N/m gives ~1.8 mm ball sag at nominal load and ~2.3 mm at the 1 kg
% payload -- small against the 50 mm radius and well inside the geometric
% accuracy budget.  Stiffer values buy nothing for the solver: the
% dominant step-size limiter in the Simscape rev1 plant is motor
% electrical dynamics, not contact stiffness.
p.contact.ballGround.normalStiffness = 3.0e4;
p.contact.ballGround.normalDamping = 500;  % ~critical for k=3e4, m~2.1 kg
p.contact.ballGround.transitionWidth = 5.0e-4;
% Ball-floor friction: real rubber-membrane ball on flooring/carpet is
% ~0.8; 1.2 keeps a modest slip margin in the model without the old
% idealized no-slip (mu=2.0) assumption.  Rolling -- not sliding -- is
% still the dominant floor interaction for a ball.
p.contact.ballGround.staticFriction = 1.2;
p.contact.ballGround.dynamicFriction = 1.2;
p.contact.ballGround.criticalVelocity = 5.0e-3;
% Ball-ground rolling resistance: a real hollow ball rolling on the
% floor loses energy through contact-patch hysteresis and scrub -- a
% horizontal drag opposing translation and a small torque on the
% vertical spin.  The point contact above resists only sliding, so
% without this term a coasting ball never sheds momentum.  Applied via
% ballbotRev1PlantAugment + ballbotRollingResistance as an external
% load scaled by the sensed normal force (vanishes when airborne).
% rollingFriction is the effective drag coefficient: hollow
% plastic/rubber balls sit around 0.005-0.02.  With the NiMH pack
% aboard, 0.008 is the value that carried a full move+stop window
% cleanly -- 0.005 let the ball overrun the command at the cmd->0
% corner (~0.08->0.24 m/s kick), 0.01 over-damped the ball (it lagged
% the command and the offset crept the contact envelope).  Deployment
% floors span hardwood to short-pile carpet (~0.005-0.015): the design
% target is robust control across that whole band, validated by sweep.
p.contact.ballGround.rollingFriction = 0.008;
p.contact.ballGround.rollingCriticalVelocity = 5.0e-3;
% spinScrubArm [m]: effective lever arm for the contact-patch scrubbing
% torque on vertical ball spin.  The patch radius at nominal sag is
% ~5 mm; half of it is a conservative arm.
p.contact.ballGround.spinScrubArm = 1.0e-3;
p.contact.ballGround.spinScrubCriticalRate = 0.5;

% Design-point overrides applied BEFORE the first derivation that
% consumes them (wheel geometry below).  Early-derived fields that a
% design knob feeds are rebuilt here so every dependent stays
% consistent.
if nargin >= 2 && ~isempty(design)
    for i = 1:numel(design)
        pair = design{i};
        f = strsplit(pair{1}, '.');
        switch numel(f)
            case 1, p.(f{1}) = pair{2};
            case 2, p.(f{1}).(f{2}) = pair{2};
            case 3, p.(f{1}).(f{2}).(f{3}) = pair{2};
            case 4, p.(f{1}).(f{2}).(f{3}).(f{4}) = pair{2};
        end
    end
    p.ball.inertia = (2/3)*p.ball.mass*p.ball.radius^2*eye(3);
    p.rover.mass = p.rover.bodyMass + ...
        3*(p.motor.mass + p.wheel.mass);
    p.rover.initialPositionWorld = [0; 0; ...
        p.ball.radius + p.rover.centerAboveBall];
end

geometry = ballbotWheelGeometry(p);
p.wheel.normalLoadNominal = ...
    p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude));
% Controller-side traction budget: keep the PHYSICAL mu of the real
% wheel-ball pair so the torque cap still encodes the wheel-slip limit,
% independent of the idealized (slip-free) contact friction above.
% 50A-durometer rubber rollers on a rubber-sprayed ball: rubber-on-rubber
% static friction is ~1.0-1.5 clean/dry (kinetic ~0.8-1.1), so 1.0 is a
% conservative design point until the slip-onset torque is measured on
% the real pair.  Revisit upward only with bench data.
p.contact.wheelBall.tractionEstimateFriction = 1.0;
p.wheel.contactTorqueLimit = p.contact.wheelBall.tractionEstimateFriction* ...
    p.wheel.normalLoadNominal*p.wheel.radius;
% Limit the actuator command by the servo capability.  The former limit
% also clipped it to the nominal wheel-ball traction torque (about
% 0.042 N*m), which prevented the controller from developing recovery
% authority.  Slip and transmissible force remain governed by the
% Spatial Contact Force blocks in the plant.
p.wheel.commandTorqueLimit = p.driver.continuousTorqueLimit;
p.wheel.geometry = geometry;

% IMU/wheel estimator. State is [q_WB(4); v_WB(3); omegaBall_W(3);
% roverMinusBall_B_xy(2); gyroBiasZ; qualificationTime].
p.estimator.sampleTime = sampleTime;
p.estimator.attitudeCorrectionGain = 2.5;
p.estimator.accelNormGate = 0.25*p.gravity;
% Reject the accelerometer up-vector correction while the body accelerates
% horizontally: the norm gate alone cannot see horizontal acceleration
% (|a| changes only at second order), so without this gate sustained
% translation biases the tilt estimate by ~atan(a/g) and the balance loop
% under-corrects until the rover falls.  The gate must sit BELOW the
% commanded acceleration (0.06 m/s^2 ramps) or the phantom lean a/g keeps
% being injected during every move window, and above the sensor noise
% floor.  An attitude error also reads as horizontal acceleration
% (g*delta), so a too-tight gate self-locks the correction at rest; the
% wheel-rate/gyro-rate quiescent override in ballbotEstimatorStep keeps
% the correction alive whenever the motion sensors prove the body is
% truly at rest.
p.estimator.accelHorizontalGate = 0.03;
p.estimator.velocityLeak = 0.9995;
p.estimator.ballRateTimeConstant = 0.080;
% Leaky integrator, ~1.5 s fold time: the relative-velocity difference
% of two drifting estimates accumulates a phantom offset (~1 cm in 4 s
% measured at tau=25 s); tau=1.5 s bounds the phantom near ~1 mm while
% real ball-escape offsets (which persist for seconds) still pass.
p.estimator.relativePositionLeak = 0.9967;
% Velocity anchor [s]: the accel-integrated rover velocity has no
% corrector during motion, so a small sustained accel bias integrates
% into a large phantom velocity (observed ~-0.36 m/s while the rover
% was nearly still).  Pull velocityWorld toward the wheel-derived
% ballVelocityWorld with this time constant while the assembly is not
% still; the rover-ball relative speed stays ~mm-scale while contacts
% hold, so the anchor drains integrator bias without hiding real
% relative motion.  0 disables it.
p.estimator.velocityAnchorTimeConstant = 0.5;
% The anchor only helps while genuinely translating; during the startup
% rocking transient the kinematic solve output is phantom and dragging
% velocityWorld toward it corrupted the estimate.  Require a clear
% moving speed first.
p.estimator.velocityAnchorMinSpeed = 0.05;
% Stillness gate for the ball-rate kinematic solve: a resting ball
% cannot rotate, so the solve target is forced to zero instead of
% amplifying quantization noise into phantom spin.  The decision runs
% on a low-passed wheel rate, not the raw flickery rate: raw rates
% under ~4.4 rad/s are ambiguous between rest flicker and real rolling
% (ball speeds below ~0.13 m/s read inside the flicker band), which
% zeroed the velocity estimate during slow translation.  Filtering at
% ~80 ms leaves rest flicker near zero while sustained rolling keeps a
% coherent rate, shrinking the blind band to ~0.03 m/s.
% removed (experiment history lives in README and git log).
% Filtered-rate stillness test: the flicker residual through a 0.15 s
% low-pass is ~+/-0.5 rad/s, so a 0.8 rad/s threshold separates rest
% from genuine rolling above ~0.02 m/s of ball speed without blanking
% slow commanded motion.
p.estimator.stillnessFilterTimeConstant = 0.15;
p.estimator.ballRateStillWheelRateFiltered = 0.8;
% Stillness velocity drain [s]: while the assembly is still, wheels roll
% nothing, so a nonzero velocityWorld is pure accelerometer-integrator
% bias.  Decay it at this time constant -- short enough to erase the
% ~0.3-0.5 m/s phantom drift that otherwise saturates the lean demand
% mid-move, long enough to smooth the encoder flicker band.
p.estimator.stillVelocityDrainTimeConstant = 0.1;
% The drain only engages after this much CONTINUOUS stillness [s]: a
% creep inside the blind band's edge (~0.025 m/s, right at the filtered
% rate threshold) flickers the flag and would otherwise have its real
% velocity drained away -- observed as a payload build that crept for
% seconds while the estimate read ~0.
p.estimator.stillVelocityDrainHoldTime = 1.0;
p.estimator.ballRateStillGyroRate = 0.15;
% Physical bound on rover-minus-ball offset [m]: ball radius scale.
p.estimator.relativePositionLimit = 0.12;
p.estimator.kinematicRegularization = 1.0e-8;

% Provisional yaw-gyro bias observer settings. These values must be tuned
% against the selected IMU noise, in-run bias stability, vibration, and
% encoder quantization before deployment on hardware.
p.estimator.biasWheelRateThreshold = 0.10;
p.estimator.biasAccelNormThreshold = 0.03*p.gravity;
p.estimator.biasRollPitchRateThreshold = 0.02;
p.estimator.biasYawRateThreshold = 0.05;
p.estimator.biasQualificationTime = 0.50;
p.estimator.biasQualificationHysteresis = 1.25;
p.estimator.biasTimeConstant = 1.0;
p.estimator.biasMaximum = 0.10;
p.estimator.biasMaximumUpdatePerSample = 1.0e-4;

% Attitude EKF (error-state, 5 dof: small-angle delta-theta + roll/pitch
% gyro bias).  The yaw gyro bias is unobservable without a heading
% sensor and stays on the qualified-stillness learner above.  The
% accelerometer update is compensated by a motion model -- the
% encoder-witnessed ball acceleration -- so gravity correction stays
% ACTIVE during motion, replacing the legacy rest-only gated
% correction that bounded drift only at stop windows.
p.estimator.ekf.enabled = true;
% Sensor noise densities, LSM6DSO datasheet high-performance mode:
% gyro 3.8 mdps/sqrt(Hz); accel 75 ug/sqrt(Hz) at the +-4 g range.
% The filter runs an inflated gyro figure (3x datasheet): it doubles
% as process noise for the unmodeled flex/mount dynamics and keeps the
% attitude covariance high enough that the Kalman gain tracks the
% plant's real tilt excursions (measured: datasheet-exact gains lagged
% the true tilt ~2.5x and the loop self-excited).
p.estimator.ekf.gyroNoiseDensity = 3.8e-3*pi/180*3;    % rad/s/sqrt(Hz)
p.estimator.ekf.accelNoiseDensity = 75e-6*p.gravity;   % m/s^2/sqrt(Hz)
% Gyro bias random walk: LSM6DSO publishes no explicit ARW; the
% zero-rate temp drift 0.010 dps/degC bounds slow wander, so allow a
% ~0.012 dps/sqrt(s) walk -- enough to track thermal bias over ~100 s
% without letting the accel update steer it fast.
p.estimator.ekf.biasRandomWalk = 2.0e-4;               % rad/s/sqrt(s)
% Residual accel noise while the in-motion update window is open:
% the chassis is provably near-inertial then, so the sigma covers
% accel noise, contact jitter, and the relPos wobble the gravity
% model ignores.  At standstill only noise + mount misalignment.
p.estimator.ekf.modelSigmaMove = 0.15;                 % m/s^2
p.estimator.ekf.modelSigmaStatic = 0.05;               % m/s^2
% Ball-velocity derivative low-pass: raw encoder flicker
% differentiates hot (~+-20 m/s^2 raw); 0.25 s leaves ~+-0.1 residue.
p.estimator.ekf.accelFilterTimeConstant = 0.25;        % s
% relPos-acceleration low-pass for the slosh witness: the wobble runs
% at ~1.5 Hz; 0.08 s keeps the witness responsive without flicker.
p.estimator.ekf.relAccFilterTimeConstant = 0.08;       % s
% Gyro-rate derivative low-pass for the tip-onset witness: raw gyro
% noise differentiates to ~+-0.4 rad/s^2 at 200 Hz; 0.10 s stays fast
% enough to catch ramp-onset tipping within a few samples.
p.estimator.ekf.gyroDotFilterTimeConstant = 0.10;      % s
% Inertial-segment gate on |predicted chassis accel| [m/s^2]: the
% prediction = a_ball + relPos_ddot + z*thetaddot; near zero means
% parked or steady cruise, where the specific force is pure gravity
% and the gravity update is valid IN MOTION.  Above the bound
% (ramps, slosh, braking) the update waits -- attitude rides on the
% gyro, unbiased over the few seconds a transient lasts.
p.estimator.ekf.accelGate = 0.15;                      % m/s^2
% Instant-onset witness [rad/s]: every accel channel is filtered and
% lags a tip onset ~0.1-0.2 s; the raw roll/pitch rate does not.
% Creep and cruise keep |w_rp| below ~0.1 while any real tip exceeds
% it within samples.
p.estimator.ekf.gyroRateGate = 0.15;                   % rad/s
% Debounce on the inertial-segment gate [s]: all accel witnesses are
% filtered and lag a ramp onset ~0.1-0.2 s; requiring the quiet
% condition to persist before updates resume removes every transient
% leak while leaving sustained cruise fully correctable.
% Wheel-slew witness: motors drive the wheels the instant a leg
% starts -- the only channel reacting before the filtered accel
% witnesses.  Quantization flicker differentiates hot raw, so the
% slew is low-passed at 0.05 s and gated above the flicker residue.
p.estimator.ekf.wheelSlewFilterTimeConstant = 0.05;    % s
p.estimator.ekf.wheelSlewGate = 3.0;                   % rad/s^2
p.estimator.ekf.inertialSettleTime = 0.30;             % s
% Per-update cap on the injected attitude step [rad]: bounds how much
% a leaked update can inject before the filtered accel witnesses
% close the gate (~0.1-0.2 s after onset).  Sized at the legacy
% corrector's 0.5 rad/s rate-clamp equivalent (0.004 rad per 5 ms
% sample); the earlier 0.02 rad cap let a single onset window inject
% ~0.03 rad of phantom tilt (offline-measured), which then stayed
% frozen into the estimate while the loop ran the tilt away.
p.estimator.ekf.maxTiltStep = 0.004;                   % rad
% Roll/pitch gyro-bias states in the error-state vector.  Set 0 while
% validating: a wrongly-accumulated bias subtracts real rate from the
% gyro and the integrated tilt under-reads exactly when it matters.
p.estimator.ekf.biasEnabled = 1;                       % 0/1
% A/B experiment: 1 runs the proven legacy gated correction inside the
% EKF predict path, isolating whether failures come from the Kalman
% update or elsewhere in the EKF wiring.
p.estimator.ekf.useLegacyCorrection = 0;               % 0/1
% Extreme-residual veto [m/s^2]: with the witness gate open the
% residual IS attitude error however large, so vetoing at the
% accel-level scale deadlocked the estimate (once lag passed ~0.03
% rad every update was rejected and the rover fell with a frozen
% estimate).  1.5 m/s^2 ~ 0.15 rad is beyond any plausible one-sample
% attitude error; larger means sensor fault or a slipping/airborne
% contact the still-reading encoders cannot see.
p.estimator.ekf.residualGate = 1.50;                   % m/s^2
% Attitude-covariance floor [rad^2]: keeps the Kalman gain alive --
% without it P collapses to the gyro-noise floor within ~1 s and the
% estimate lags real tilt excursions.  (5 mrad)^2 ~ gain bandwidth
% ~0.1 s at the static R.
p.estimator.ekf.tiltVarFloor = (5.0e-3)^2;             % rad^2
p.estimator.ekf.initTiltStd = 0.05;    % rad, startup alignment
p.estimator.ekf.initBiasStd = 0.02;    % rad/s, LSM6DSO zero-rate +-1 dps

% Cascaded velocity/balance/yaw controller.
p.controller.sampleTime = p.estimator.sampleTime;
% Rev2 multi-rate: inner wheel servo rate.  The STM32G474RE (170 MHz
% M4F) schedules this as a faster discrete task under R2024b Embedded
% Coder's "treat each discrete rate as separate task" mode.
p.controller.servoSampleTime = 0.001;
p.controller.velocityKp = [0.35; 0.35];
p.controller.velocityKi = [0.04; 0.04];
p.controller.relativePositionKp = [0; 0];
p.controller.velocityIntegralLimit = [0.20; 0.20];
p.controller.velocityIntegralLeak = 0.995;
p.controller.velocityConvergenceBand = 0.005;
p.controller.tiltPriorityStart = deg2rad(3);
p.controller.maxLean = deg2rad(4);
p.controller.balanceKp = [0.95; 0.95];
p.controller.balanceKd = [0.12; 0.12];
p.controller.yawRateKp = 0.08;
% Keep yaw actuation inhibited during startup calibration until the
% corrected yaw rate is within the provisional acceptance tolerance.
p.controller.yawBiasReadyRateThreshold = 0.002;
% A deliberate yaw command bypasses startup inhibition; low-motion bias
% learning remains disabled naturally once the wheels move.
p.controller.yawBiasCommandBypassThreshold = 1.0e-6;
p.controller.maxPlanarAcceleration = 0.60;
p.controller.maxYawRate = 0.80;
% Coupled (v, yawRate) feasibility limiter: all three wheels share the
% motor-speed budget, so the command pair's wheel-rate demand
% sin(lat)*|v| + Rch*|yawRate| is scaled jointly down to Rw*wMotorLimit
% when the pair exceeds it, instead of clipping each axis separately.
p.controller.coupledCommandLimitEnabled = true;
% Recovery entry threshold [rad]: covers tilted starts (hardware
% placement is specified <10 deg).  Normal-cycle transients stay
% below ~3 deg, so 5 deg separates an emergency lean from ordinary
% move/stop excursions while still engaging early enough to arrest a
% 10 deg start (validated: rights to <2 deg in ~0.4 s).
p.controller.recoveryTilt = deg2rad(5);
% Recovery exit threshold (hysteresis): mode 2 holds until the tilt
% drops below this, so the higher traction allowance still covers the
% post-righting drift-arrest phase instead of handing a moving
% assembly back to the derated normal budget.
p.controller.recoveryExitTilt = deg2rad(2);
% Elevated-authority clear rate [rad/s wheel]: after mode 2 hands off
% to mode 1, the recovery traction budget persists until the filtered
% wheel rate drops below this -- post-recovery drift arrest needs the
% larger cap.  Above the encoder flicker floor (~3.7 rad/s raw, but
% the low-passed signal settles near zero when parked).
p.controller.recoveryAuthorityClearRate = 1.5;
% (recoveryRightTime / ramped tilt reference removed: a time-ramped
% reference cannot arrest the initial fall -- the tilt error starts at
% ~0 exactly when maximum torque is needed, and the 10 deg start fell
% inside ~1-3 s for every direction.)
p.controller.fallenTilt = deg2rad(35);
% Recovery gain multiplier over the LQI state gain: the nominal gain
% only requests ~0.2 N*m at a 10 deg lean vs the ~0.44 N*m gravity
% topple moment; 3x reaches the traction allowance and arrests the
% fall.
p.controller.recoveryGainScale = 3.0;

% Upright, centered, no-slip command limits. For pure translation the
% worst-case wheel rate is sin(lambda)*v/Rw. At half the resulting
% no-load-speed limit the FIT0521 torque-speed envelope still exceeds the
% wheel-ball traction limit, so traction limits acceleration, not speed.
p.controller.theoreticalMaxSpeed = p.wheel.radius* ...
    p.motor.speedCommandLimit/sin(p.wheel.contactLatitude);
p.controller.commandSpeedFraction = 0.50;
p.controller.targetCommandSpeed = p.controller.commandSpeedFraction* ...
    p.controller.theoreticalMaxSpeed;
p.controller.maxSpeed = p.controller.targetCommandSpeed;
p.controller.tractionLimitedWheelTorque = min( ...
    p.driver.continuousTorqueLimit, p.wheel.contactTorqueLimit);
% Recovery-mode traction allowance [N*m]: mode=2 is allowed more of
% the contact's real friction than the derated normal-operation budget
% -- a tilted start needs ~0.44 N*m about the ball centre while the
% nominal cap only supplies ~0.49 N*m ideal (less once a wheel
% unloads).  2x the nominal cap rights 10 deg starts without slip;
% ~3x+ overdrives into slip.  ballbotRev1Parameters applies the same
% ratio on top of its mass-scaled nominal cap.
p.controller.recoveryWheelTorqueLimit = ...
    2.0*p.controller.tractionLimitedWheelTorque;
p.controller.theoreticalTractionAcceleration = sqrt(3)* ...
    p.controller.tractionLimitedWheelTorque/(p.wheel.radius* ...
    (p.rover.mass + p.ball.mass));
p.controller.theoreticalLeanAcceleration = ...
    p.gravity*tan(p.controller.maxLean);
p.controller.theoreticalCommandAcceleration = min([ ...
    p.controller.maxPlanarAcceleration, ...
    p.controller.theoreticalTractionAcceleration, ...
    p.controller.theoreticalLeanAcceleration]);

% Ten-state MIMO LQI with the physical three-wheel geometry at the
% configured wheel.contactLatitude (33 deg in the current design).
rollingXY = p.wheel.geometry.rollingBody(1:2, :).';
p.controller.lqi.wheelSpeedFromPlanarVelocity = [ ...
    -(sin(p.wheel.contactLatitude)/p.wheel.radius)*rollingXY, ...
    (p.rover.chassisRadius/p.wheel.radius)*ones(3, 1)];
p.controller.lqi.maximumState = [p.controller.maxSpeed; ...
    p.controller.maxSpeed; p.controller.maxLean; p.controller.maxLean; ...
    1.5; 1.5; p.controller.maxYawRate; ...
    p.motor.speedCommandLimit*ones(3, 1)];
p.controller.lqi.maximumVelocityIntegral = [0.12; 0.12];
p.controller.lqi.maximumWheelSpeedCommand = ...
    0.70*p.motor.speedCommandLimit*ones(3, 1);
p.controller.lqi.linearizationStateStep = [1.0e-5; 1.0e-5; ...
    1.0e-6; 1.0e-6; 1.0e-5; 1.0e-5; 1.0e-5; ...
    1.0e-4; 1.0e-4; 1.0e-4];
p.controller.lqi.linearizationInputStep = 1.0e-4*ones(3, 1);
p.controller.lqi.antiWindupGain = 8.0;
p.controller.lqi.trackingGainScale = 1.0e-4;
p.controller.lqi = ballbotDesignLqi(p, p.controller.lqi);

% Upright PID and direct velocity-to-torque controller used by
% ball_balancing_omni3_multibody_pid.slx. The tilt reference is fixed at
% zero; commanded planar velocity contributes torque directly and does not
% generate a lean-angle reference.
p.controller.pidTiltKp = [0.95; 0.95];
p.controller.pidTiltKi = [0.02; 0.02];
p.controller.pidTiltKd = [0.12; 0.12];
p.controller.pidTiltIntegralLimit = deg2rad([5; 5]);
p.controller.pidVelocityKp = [0.03; 0.03];
p.controller.pidYawRateKp = 0.08;

p.command.velocityWorld = [p.controller.targetCommandSpeed; 0];
p.command.yawRate = 0;
p.command.enable = true;

p.simulation.stopTime = 4.0;
p.simulation.maxStep = 1.0e-4;
p.simulation.relativeTolerance = 1.0e-4;
p.simulation.imuYawBias = 0;
% Non-ideal floor: floorSlopeDeg inclines the contact plane
% (applied to Environment/GroundFrame at runtime by
% ballbotRev1PlantAugment); floorSlopeAzimuthDeg chooses the descent
% azimuth (0 -> downhill is +x).  0 = ideally level floor.  Measured
% envelope: 1 deg holds, >=2 deg rolls the assembly out of balance.
p.simulation.floorSlopeDeg = 0;
p.simulation.floorSlopeAzimuthDeg = 0;
end
