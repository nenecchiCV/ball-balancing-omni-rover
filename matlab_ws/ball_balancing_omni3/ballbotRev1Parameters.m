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
    [p.rover.mass, p.rover.comAboveBall, p.rover.tiltInertiaCom] = ...
        ballbotPayloadRoverProperties(p, payloadMass);
    p.fullplant.payload.comAboveChassis = ...
        p.rover.comAboveBall - p.rover.centerAboveBall;
    p.wheel.normalLoadNominal = ...
        p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude));
    p.wheel.contactTorqueLimit = p.contact.wheelBall.dynamicFriction* ...
        p.wheel.normalLoadNominal*p.wheel.radius;
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
    % Rover origin is the CHASSIS frame, which sits centerAboveBall above
    % the ball centre regardless of where the payload moved the combined
    % COM.  Using comAboveBall here lifted the chassis ~36-68 mm and the
    % build free-dropped onto the ball: wheels bounced off the contact
    % (normal force impulses of several kN, then intermittent contact),
    % which is the observed hopping and seeded the moving-start divergences.
    p.rover.initialPositionWorld(3) = p.ball.radius + ...
        p.rover.centerAboveBall - p.fullplant.groundStaticDeflection;
end
% The assembled model's logged wheel-ball normal force is ~4.07 N at
% rest, not mg/(3 sin(lat)) = 5.44 N.  Worse, the normal force dips ~50%
% during transients, so the clamp must sit well under the nominal
% dynamic slip threshold (mu*N*r = 0.073 N*m) or a torque command spins
% the wheel free and the motor reaction tips the chassis.  Applies to
% the default call too, not only payload sweeps.  The measured rest load
% scales with supported mass, so the limit does too: heavier builds get
% the larger traction budget their normal force can actually transmit,
% which is what gives heavy configurations the torque headroom to leave
% a static lean instead of stalling.
p.controller.tractionLimitedWheelTorque = min( ...
    p.driver.continuousTorqueLimit, ...
    0.55*p.contact.wheelBall.dynamicFriction*4.07* ...
    (p.rover.mass/(p.fullplant.payload.originalRoverMass + 0.50))* ...
    p.wheel.radius);

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
% The move window must cover the lean-limited ramp to slip-limit speed
% plus a cruise hold: 0.524 m/s / 0.06 m/s^2 ~ 8.7 s ramp + ~1.3 s hold.
p.command.cycleMoveDuration = 10.0;
% Yaw control is fully separated from translation: the yaw-rate request
% is issued as a trapezoid inside the stop window, starting cycleYawDelay
% seconds after motion stops.  The stationary yaw gate in the controller
% opens only while command speed is zero, so the profile must guarantee
% the yaw window lies inside a stop phase with margin on both sides.
p.command.cycleStopDuration = 4.0;
p.command.cycleYawDelay = 1.0;
p.command.cycleYawDuration = 1.5;
p.command.cycleCount = 3;
% The holdable lean cone is ~0.023 rad of tilt at the full traction
% budget, and propulsion torque spends part of that budget during the
% ramp.  Commanded acceleration is therefore capped well inside the
% cone: 0.06 m/s^2 ~ lean 0.006 rad, about a quarter of the limit.
p.command.accelerationLimit = 0.06;

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

% Measured-plant gains, designed robustly over the payload family instead
% of at the nominal mass: ballbotRobustTorqueGains scales the measured
% torque-input B matrix (open-loop steps on the nominal build) across
% payload 0..1 kg via 1/Ipivot and 1/Iyaw, solves one discrete LQR per
% mass, and keeps the candidate whose worst-case closed-loop pole radius
% over the whole family is smallest.  A nominal-point design is NOT
% robust here: per-torque tilt authority falls ~8x from the light to the
% heavy end, so the fixed gain must come from a heavy-end design; the
% torque clamp bounds the resulting stiffness on light builds, whose
% authority margin covers the saturation.  The empirical column scalings
% are preserved (tilt x4 against the stiction/quantization floor, tilt-rate
% x0.125 against the ~9 Hz electrical-lag ring, yaw rate x5).  Velocity and
% wheel-rate columns stay zero: velocity tracking in torque mode has no
% feedforward yet, and wheel-rate feedback re-injects encoder
% quantization.
[p.controller.torqueLqi.stateGain, p.controller.torqueLqi.robustness] = ...
    ballbotRobustTorqueGains(p);
% Stationary yaw gate [SI]: the yaw-rate feedback column and the yaw-rate
% command are applied only while the rover is near rest.  commandSpeed
% must be exactly zero so a move window always closes the gate.
p.controller.torqueDrive.yawGate.maxSpeed = 0.03;
p.controller.torqueDrive.yawGate.maxTilt = 0.02;
p.controller.torqueDrive.yawGate.commandThreshold = 1.0e-3;
p.controller.torqueDrive.yawGate.rateThreshold = 0.05;
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
% Enabled: the yaw-damper/ball-brake pump is now removed and the inner
% loop holds machine-zero tilt, so the lean channel is the only physical
% translation actuator.  leanSetpointLimit must sit well inside the
% measured torque-limited hold cone (~0.023 rad): a 0.02 rad demand at
% 87% of the cone tipped at t ~1 s because any tilt overshoot past the
% cone is unrecoverable (gravity pole ~6.4 rad/s, ~150 ms e-folding).
% 0.008 rad covers the 0.06 m/s^2 ramp demand (~0.006 rad) with ~35%
% margin for transient overshoot.
p.controller.torqueDrive.velocityLeanGain = 0.3;
p.controller.torqueDrive.leanSetpointLimit = 0.008;
% Tilt overshoot guard for the lean channel: once the measured tilt
% exceeds the demanded lean by leanFadeTilt the demand fades to zero, so
% a cone-exceeding excursion stops adding forward acceleration bias.
% 0.02 rad starts fading just past the ~0.023 rad hold cone; tilt inside
% the commanded band passes unattenuated.
p.controller.torqueDrive.leanFadeTilt = 0.02;
% Station-keeping term on the lean channel: relPos = rover - ball.
% During commanded acceleration the ball must be torqued up through the
% wheel contacts; if the chassis leads the ball (relPos > 0) the lean
% demand must back off or the rover runs away from its support and
% faceplants.  P without D oscillates (measured), so the channel needs
% the relative-velocity damping term.
p.controller.torqueDrive.positionLeanGain = 0;
p.controller.torqueDrive.positionLeanDampingGain = 0;
% Propulsion: P servo tracking the kinematic wheel rate W*v_cmd, in
% N*m per rad/s of wheel-rate error.  The tilt loop handles attitude;
% this term only drives the wheel/ball rolling state toward the command.
p.controller.torqueDrive.propulsionGain = 0.005;
% Propulsion torque share cap [N*m]: without a bound the initial rate
% error (~20 rad/s for a 0.5 m/s command) saturates one wheel's duty to
% ~0.95 while the tilt channel starves.  Half the traction limit leaves
% headroom for attitude correction during acceleration.
p.controller.torqueDrive.propulsionTorqueLimit = 0.5* ...
    p.controller.tractionLimitedWheelTorque;
% Stall escape: when the tilt channel saturates the traction budget the
% leftover-headroom policy gives propulsion zero torque forever -- the
% rover wedges in a static lean (observed at high payload: ~0.14 rad
% held, wheels stalled, ball static).  A minimum propulsion share lets
% the drive pattern break the frictional equilibrium; the overshoot above
% the nominal clamp is bounded by this share.
p.controller.torqueDrive.propulsionMinShare = 0.25;
% Relative-position closure on the wheel-rate command [1/s] and
% relative-velocity damping [dimensionless]: the ball has to be torqued
% up by the wheels, so a chassis lead is corrected by spinning the
% ball faster.  Lean-channel closure of the same loop was tried and
% oscillated; the wheel rate path is faster and kinematically direct.
p.controller.torqueDrive.propulsionRelPosGain = 0;
p.controller.torqueDrive.propulsionRelVelGain = 0;
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
% brake was expected to damp that mode, but the A/B truth-feedback
% experiment showed the yaw-damper + brake pair still pumps the
% yaw/ball-spin mode to tip-over (~5 s), while disabling both holds
% machine-zero tilt ~10.5 s -- so it ships at 0.
p.controller.torqueDrive.ballYawGain = 0;

% Diagnostic: feed plant truth (roverPose/ballPose) through TruthGate
% instead of the estimator output, to separate estimator error from
% plant/controller physics in divergence analysis.  truthChannels
% optionally restricts the substitution to a subset of the 14-element
% estimate vector ([] = all channels).  Model default off.
p.estimator.truthFeedbackEnabled = false;
p.estimator.truthChannels = [];
% Artificial latency on the truth path (control samples).  Used to test
% whether the estimator's disadvantage is estimation error or simply
% the one-step update latency of the estimator pipeline.
p.estimator.truthDelaySteps = 0;

% Identifies which builder produced this set so the rev1 model InitFcn
% can regenerate ballbotParams when a stale variant (which lacks the
% electrical actuator fields the Simscape network requires) is left in
% the workspace.
p.parameterSource = "ballbotRev1Parameters";
end
