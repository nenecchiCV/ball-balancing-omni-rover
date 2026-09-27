function p = ballbotRev1Parameters(payloadMass)
%BALLBOTREV1PARAMETERS Parameters for the rev1 Simscape-actuator model.
%   p = BALLBOTREV1PARAMETERS returns the parameters for
%   ball_balancing_omni3_multibody_lqi_rev1.slx: the full-plant parameter
%   set extended with the electrical actuator quantities that the Simscape
%   motor/driver network requires.
%
%   p = BALLBOTREV1PARAMETERS(payloadMass) replaces the mounted payload
%   mass [kg] and recomputes every payload-dependent plant quantity.  The
%   LQI and PI gains intentionally stay at the nominal design so the sweep
%   measures robustness to an unannounced load change.

p = ballbotFullPlantParameters;

if nargin >= 1 && ~isempty(payloadMass)
    p.fullplant.payload.mass = payloadMass;
    wheelCenterAboveBall = p.wheel.geometry.centerFromBall(3, 1);
    baseComAboveBall = (p.rover.bodyMass*p.rover.bodyComAboveBall + ...
        3*(p.motor.mass + p.wheel.mass)*wheelCenterAboveBall)/ ...
        p.fullplant.payload.originalRoverMass;
    payloadComAboveBall = p.rover.centerAboveBall + ...
        p.fullplant.payload.centerOffset;
    p.rover.mass = p.fullplant.payload.originalRoverMass + payloadMass;
    p.rover.comAboveBall = (p.fullplant.payload.originalRoverMass* ...
        baseComAboveBall + payloadMass*payloadComAboveBall)/p.rover.mass;
    p.fullplant.payload.comAboveChassis = ...
        p.rover.comAboveBall - p.rover.centerAboveBall;
    p.rover.tiltInertiaCom = p.rover.bodyInertia(1) + ...
        p.rover.bodyMass*(p.rover.bodyComAboveBall - ...
        p.rover.comAboveBall)^2 + ...
        3*(p.motor.mass*(3*0.0122^2 + 0.052^2)/12 + ...
           p.motor.mass*(wheelCenterAboveBall - p.rover.comAboveBall)^2 + ...
           p.wheel.mass*p.wheel.radius^2/2 + ...
           p.wheel.mass*(wheelCenterAboveBall - p.rover.comAboveBall)^2) + ...
        payloadMass*p.fullplant.payload.length^2/12 + ...
        payloadMass*(payloadComAboveBall - p.rover.comAboveBall)^2;
    p.wheel.normalLoadNominal = ...
        p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude));
    p.wheel.contactTorqueLimit = p.contact.wheelBall.dynamicFriction* ...
        p.wheel.normalLoadNominal*p.wheel.radius;
    % The assembled model's logged wheel-ball normal force is ~4.07 N at
    % rest, not mg/(3 sin(lat)) = 5.44 N.  Worse, the normal force dips
    % ~50% during transients, so the clamp must sit well under the
    % nominal dynamic slip threshold (mu*N*r = 0.073 N*m) or a torque
    % command spins the wheel free and the motor reaction tips the
    % chassis.
    p.controller.tractionLimitedWheelTorque = min( ...
        p.driver.continuousTorqueLimit, ...
        0.55*p.contact.wheelBall.dynamicFriction*4.07*p.wheel.radius);
    p.controller.theoreticalTractionAcceleration = sqrt(3)* ...
        p.controller.tractionLimitedWheelTorque/(p.wheel.radius* ...
        (p.rover.mass + p.ball.mass));
    p.controller.theoreticalCommandAcceleration = min([ ...
        p.controller.maxPlanarAcceleration, ...
        p.controller.theoreticalTractionAcceleration, ...
        p.controller.theoreticalLeanAcceleration]);
    supportedMass = p.ball.mass + p.rover.mass;
    p.fullplant.linearSpringDeflection = supportedMass*p.gravity/ ...
        p.contact.ballGround.normalStiffness;
    % Scale the empirically fitted static deflection with the supported
    % mass; the reference value was measured at the nominal payload.
    nominalSupportedMass = p.ball.mass + ...
        p.fullplant.payload.originalRoverMass + 0.50;
    p.fullplant.groundStaticDeflection = 1.43e-4* ...
        supportedMass/nominalSupportedMass;
    p.rover.initialPositionWorld(3) = p.ball.radius + ...
        p.rover.comAboveBall - p.fullplant.groundStaticDeflection;
end
% The assembled model's logged wheel-ball normal force is ~4.07 N at
% rest, not mg/(3 sin(lat)) = 5.44 N.  Worse, the normal force dips ~50%
% during transients, so the clamp must sit well under the nominal
% dynamic slip threshold (mu*N*r = 0.073 N*m) or a torque command spins
% the wheel free and the motor reaction tips the chassis.  Applies to
% the default call too, not only payload sweeps.
p.controller.tractionLimitedWheelTorque = min( ...
    p.driver.continuousTorqueLimit, ...
    0.55*p.contact.wheelBall.dynamicFriction*4.07*p.wheel.radius);

% Armature inductance is not in the FIT0521 datasheet.  A typical 6 V
% micro-gearmotor of this size has an electrical time constant well below
% 1 ms; 1 mH gives tau_e = 0.53 ms against the 30 ms mechanical constant.
p.motor.armatureInductance = 1.0e-3;

% MDD3A logic inputs accept 3.3/5 V logic.  The averaged H-Bridge maps the
% PWM input voltage 0..logicHighVoltage onto a 0..100% duty ratio and
% asserts direction reversal above the reverse threshold.
p.driver.logicHighVoltage = 5.0;
p.driver.enableThresholdVoltage = 2.5;
p.driver.reverseThresholdVoltage = 2.5;
p.driver.brakeThresholdVoltage = 2.5;
% TB67H450 H-bridge: combined high+low side on-resistance.
p.driver.bridgeOnResistance = 0.35;
p.driver.bridgeOffConductance = 1.0e-6;

% Repeated move/stop command cycle used by ballbotRev1CommandProfile.
% The planar command ramps at accelerationLimit; the controller already
% caps the lean-equivalent demand through theoreticalCommandAcceleration,
% but ramping the reference keeps the feedforward step from outrunning
% the physical lean the body can establish.
p.command.cyclesEnabled = true;
p.command.cycleStart = 0.5;
% 0.9009/0.5 = 1.80 s ramps plus a 0.90 s cruise hold at the peak.
p.command.cycleMoveDuration = 4.5;
p.command.cycleStopDuration = 1.0;
p.command.cycleCount = 3;
p.command.accelerationLimit = 0.5;

% Slip-limit cruise speed: the wheel speed where the motor torque-speed
% curve crosses the wheel-ball contact torque limit mu_d*N_nom*Rw.
% Above it the motor cannot produce enough torque to slip the contact.
p.controller.slipLimitWheelSpeed = p.motor.noLoadSpeed* ...
    (1 - p.wheel.contactTorqueLimit/p.motor.stallTorque);
p.controller.slipLimitSpeed = min( ...
    p.controller.slipLimitWheelSpeed*p.wheel.radius/ ...
    sin(p.wheel.contactLatitude), p.controller.theoreticalMaxSpeed);
p.controller.maxSpeed = p.controller.slipLimitSpeed;
% Cruise at the slip-limit speed consumes ~20.4 rad/s of wheel speed;
% allow the command to reach the full motor speed envelope.
p.controller.lqi.maximumWheelSpeedCommand = ...
    p.motor.speedCommandLimit*ones(3, 1);
% commandScale derates the planar reference, so the profile requests the
% value that leaves commandScale*command == maxSpeed.
p.command.profileVelocityWorld = [ ...
    p.controller.maxSpeed/p.controller.fullplant.commandScale; 0];
% Simultaneous yaw request, capped by the controller at maxYawRate.
p.command.profileYawRate = p.controller.maxYawRate;

% Torque-mode wheel drive.  The voltage PI closes the wheel-rate loop at
% about 6 rad/s, slower than the ~6.4 rad/s unstable tilt pole, so the
% balance loop can never catch the plant.  ballbotTorqueDutyStep instead
% takes a wheel-torque command and inverts the motor equation for the
% duty, so the fast channel responds at the electrical time scale and no
% quantized wheel-rate feedback multiplies into full-scale torque.  The
% LQI is redesigned on the torque-input plant ballbotTorquePlantDynamics.
p.controller.torqueDrive.enabled = true;
% Slip-rate damping [N*m per rad/s] applied per wheel on the excess of
% measured wheel rate over the kinematic rolling rate.  A slipping wheel
% cannot transmit its torque to the ball, so braking the slip keeps the
% contact engaged instead of letting the wheel free-spin.  slipDeadzone
% hides the encoder-quantization flicker (quantization/sampleTime), and
% backEmfSpeedTimeConstant low-passes the wheel speed used in the
% duty back-EMF term for the same reason.
p.controller.torqueDrive.slipDampingGain = 0.01;
p.controller.torqueDrive.slipDeadzone = ...
    1.2*p.encoder.quantization/p.controller.sampleTime;
% 15 ms still passes ~+-0.08 duty of encoder flicker (quantization/dt =
% 3.68 rad/s through a 5 ms blend of 0.25), which is ~0.066 N*m of
% torque noise -- above the traction clamp itself and the observed
% limit-cycle exciter.  80 ms attenuates the flicker to ~+-0.005 duty;
% the lag only slows back-EMF tracking of genuine speed changes.
p.controller.torqueDrive.backEmfSpeedTimeConstant = 0.08;
% Velocity-integral leak [1/s]: in torque mode a wound-up integral is a
% constant torque bias that tips the chassis, so it must decay unless a
% commanded velocity keeps feeding it.
p.controller.torqueDrive.integralLeak = 2.0;
% Bisection switches while the torque path is being brought up.
p.controller.torqueDrive.integralEnabled = false;
p.controller.fullplant.enabled = false;
% Open-loop torque-step fit measured on the Simscape rev1 plant (0.01 N*m
% per wheel, 0.25 s): the rigid reduced model overpredicts the tilt and
% planar coupling by ~2.4x and the yaw coupling by ~70x, and the wheels
% drag each other through the chassis.  ballbotTorquePlantDynamics
% applies these scales so the torque LQI is designed on the measured
% plant rather than the idealized one.
p.torqueFit.forceScale = 0.5;
p.torqueFit.tiltScale = 0.425;
p.torqueFit.yawScale = 0.0145;
p.torqueFit.wheelDriveMatrix = 306* ...
    [1, 0.106, 0.106; 0.106, 1, 0.105; 0.105, 0.106, 1];
p.controller.torqueLqi.maximumState = p.controller.lqi.maximumState;
p.controller.torqueLqi.maximumVelocityIntegral = ...
    p.controller.lqi.maximumVelocityIntegral;
p.controller.torqueLqi.maximumWheelTorqueCommand = ...
    p.controller.tractionLimitedWheelTorque*ones(3, 1);
p.controller.torqueLqi.linearizationStateStep = ...
    p.controller.lqi.linearizationStateStep;
p.controller.torqueLqi.linearizationInputStep = 1.0e-3*ones(3, 1);
p.controller.torqueLqi = ballbotDesignTorqueLqi(p, p.controller.torqueLqi);

% Measured-plant gains: ballbotDesignTorqueLqi designs on the reduced
% model, which leaves a slowly-growing residual mode on the assembled
% plant.  These gains come from a discrete LQR on the directly measured
% torque-input B matrix (open-loop steps through the real drive chain):
% tilt channel scaled x8 to overcome the stiction/quantization floor,
% tilt-rate channel scaled x0.25 (full dlqr rate gain rings at ~9 Hz
% because the electrical + sampling lag is not in the plant model), yaw
% rate scaled x5 for tracking authority.  velocity and wheel-rate
% columns stay zero: velocity tracking in torque mode has no feedforward
% yet, and wheel-rate feedback re-injects encoder quantization.
measuredB = [ ...
    0.0520, -23.5629, 23.5170; ...   % pitch accel  [rad/s^2 per N*m]
    -27.1781, 13.5441, 13.6378; ...   % roll accel
    22.7010, 22.6692, 22.7372; ...    % yaw accel
    305.9042, 32.1404, 32.7251; ...   % wheel1 accel
    32.7227, 305.9042, 32.1425; ...   % wheel2 accel
    32.1404, 32.7227, 305.9652];      % wheel3 accel
unstableTiltPole2 = 41;              % ~(m g h / I) on the rev1 build
amp = zeros(8);                      % [pitch, -roll, q, -p, r, w1..w3]
amp(1, 3) = 1; amp(2, 4) = 1;
amp(3, 1) = unstableTiltPole2; amp(4, 2) = unstableTiltPole2;
amp(6, 6) = -1.35; amp(7, 7) = -1.35; amp(8, 8) = -1.35;
bmp = zeros(8, 3);
bmp(3, :) = measuredB(1, :);         % pitch accel
bmp(4, :) = -measuredB(2, :);        % d(-roll)/dt = -p
bmp(5, :) = measuredB(3, :);         % yaw accel
bmp(6:8, :) = measuredB(4:6, :);     % wheel accels
plantDiscrete = c2d(ss(amp, bmp, eye(8), zeros(8, 3)), ...
    p.controller.sampleTime);
KMeasured = dlqr(plantDiscrete.A, plantDiscrete.B, ...
    diag([200, 200, 20, 20, 5, 0.2, 0.2, 0.2]), 2000*eye(3));
K10 = zeros(3, 10);
K10(:, 3:4) = 8*KMeasured(:, 1:2);     % tilt [pitch, -roll]
K10(:, 5:6) = 0.25*KMeasured(:, 3:4);  % tilt rate [q, -p]
K10(:, 7) = 5*KMeasured(:, 5);         % yaw rate
p.controller.torqueLqi.stateGain = K10;
p.controller.torqueLqi.integralGain = zeros(3, 2);
p.controller.torqueDrive.slipDampingGain = 0.02;
% Outer loop.  Two structurally separate channels, because wheel torque
% is an internal force of the rover+ball system: it cannot accelerate
% the assembly centre of mass, so velocity tracking has to go through
% gravity (a lean setpoint), while the rover-ball offset IS an internal
% degree of freedom and takes a direct torque through the wheel map.
% Channel 1, velocity -> lean: state(3:4) is shifted by
% [aX; -aY]/g where a = velocityLeanGain*(cmd - v).  velocityLeanGain
% [1/s] sets the outer bandwidth (~1 rad/s, well under the ~6 rad/s
% tilt loop), and leanSetpointLimit [rad] keeps the commanded lean
% inside the recoverable cone.
% Disabled by default: with the inner loop still tipping at ~4-7 s,
% the lean channel cannot be validated in isolation.  A 0.09 m/s
% command test diverged at ~2.3 s, indistinguishable from the baseline
% instability.  Enable once the inner loop holds past tens of seconds.
p.controller.torqueDrive.velocityLeanGain = 0;
p.controller.torqueDrive.leanSetpointLimit = 0.035;
% Channel 2, station keeping: tau += W*(positionGain*s +
% positionDampingGain*sDot).  The open-loop offset dynamics measure
% s2 ~= -8.7*c [m/s^2 per N*m coefficient], but inside the tilt loop the
% PD converts most of an applied torque into lean, so the quasi-static
% effective gain is small and inverted; pending gains need a coupled
% design, so this channel ships disabled.
p.controller.torqueDrive.positionGain = 0;
p.controller.torqueDrive.positionDampingGain = 0;
% Ball vertical-spin brake [N*m per rad/s of omegaBall_z]: the rover-yaw
% damper sheds chassis angular momentum through common-mode wheel
% torque, which lands on the ball's uncontrolled spin mode and spins it
% up ~1 rad/s^2 until the contacts need lateral slip and let go.  The
% common-mode channel couples at ~22.7 rad/s^2 per N*m, so 0.02 gives
% ~0.45/s damping at the cost of up to 0.07 N*m per wheel near the
% runaway point (clamped by the traction limit).
p.controller.torqueDrive.ballYawGain = 0.01;

% Identifies which builder produced this set so the rev1 model InitFcn
% can regenerate ballbotParams when a stale variant (which lacks the
% electrical actuator fields the Simscape network requires) is left in
% the workspace.
p.parameterSource = "ballbotRev1Parameters";
end
