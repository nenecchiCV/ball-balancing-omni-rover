function p = ballbotRev1Parameters(payloadMass)
%BALLBOTREV1PARAMETERS Parameters for the rev1 Simscape-actuator model.
%   p = BALLBOTREV1PARAMETERS returns the parameters for
%   ball_balancing_omni3_multibody_lqi_rev1.slx: the full-plant parameter
%   set extended with the electrical actuator quantities that the Simscape
%   motor/driver network requires.
%
%   p = BALLBOTREV1PARAMETERS(payloadMass) replaces the mounted payload
%   mass [kg] and recomputes every payload-dependent plant quantity.  The
%   legacy LQI and PI gains intentionally stay at the nominal design; the
%   cascade LQR is payload-SCHEDULED instead (its hold floor and the
%   traction saturation cone do not intersect for a single gain across
%   the family, and the mounted payload is physically known anyway).

p = ballbotFullPlantParameters;

if nargin >= 1 && ~isempty(payloadMass)
    p.fullplant.payload.mass = payloadMass;
    [p.rover.mass, p.rover.comAboveBall, p.rover.tiltInertiaCom] = ...
        ballbotPayloadRoverProperties(p, payloadMass);
    p.fullplant.payload.comAboveChassis = ...
        p.rover.comAboveBall - p.rover.centerAboveBall;
    p.wheel.normalLoadNominal = ...
        p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude));
    p.wheel.contactTorqueLimit = p.contact.wheelBall.tractionEstimateFriction* ...
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
    % Static deflection is the analytic spring sag under the supported
    % mass (the previous 1.43e-4 m constant was an empirical fit for the
    % old 3e5 N/m stiffness and does not transfer across stiffness).
    p.fullplant.groundStaticDeflection = p.fullplant.linearSpringDeflection;
    % Rover origin is the CHASSIS frame, which sits centerAboveBall above
    % the ball centre regardless of where the payload moved the combined
    % COM.  Using comAboveBall here lifted the chassis ~36-68 mm and the
    % build free-dropped onto the ball: wheels bounced off the contact
    % (normal force impulses of several kN, then intermittent contact),
    % which is the observed hopping and seeded the moving-start divergences.
    p.fullplant.wheelBallStaticDeflection = p.wheel.normalLoadNominal/ ...
        p.contact.wheelBall.normalStiffness/sin(p.wheel.contactLatitude);
    p.rover.initialPositionWorld(3) = p.ball.radius + ...
        p.rover.centerAboveBall - p.fullplant.groundStaticDeflection - ...
        p.fullplant.wheelBallStaticDeflection;
end
% The assembled model's logged wheel-ball normal force is ~4.07 N at
% rest, not mg/(3 sin(lat)) = 5.44 N.  The clamp sits at 80% of the
% nominal slip threshold: with the idealized (critically damped,
% slip-free) contacts the transient N-dips are modest, so the earlier
% 0.55 derating was overcautious.  Applies to the default call too, not
% only payload sweeps.  The measured rest load scales with supported
% mass, so the limit does too: heavier builds get the larger traction
% budget their normal force can actually transmit, which is what gives
% heavy configurations the torque headroom to leave a static lean
% instead of stalling.
p.controller.tractionLimitedWheelTorque = min( ...
    p.driver.continuousTorqueLimit, ...
    0.80*p.contact.wheelBall.tractionEstimateFriction*4.07* ...
    (p.rover.mass/(p.fullplant.payload.originalRoverMass + 0.50))* ...
    p.wheel.radius);

% Armature inductance is not in the FIT0521 datasheet; motors of this
% class typically land in the 0.5-5 mH range.  5 mH gives tau_e = 2.7 ms
% against the 30 ms mechanical constant -- still a fast electrical state,
% but slow enough that the variable-step solver stops failing Newton
% iterations on the armature current (with 1 mH the i_L states produced
% ~600 of the residual solver exceptions per 2 s run; at 5 mH that drops
% to ~180 and total steps fall ~30%).
p.motor.armatureInductance = 5.0e-3;
% Effective actuator lag seen by the balance loop [s]: dominated by the
% motor's electromechanical time constant (30 ms in the base motor
% spec); the ~3 ms armature lag and 5 ms discrete ZOH are absorbed
% into the same first-order model.  The cascade LQR design augments the
% plant with a first-order lag of this value and feeds back the tracked
% applied torque, so phase loss is designed in rather than discovered
% in the sim.
p.motor.actuatorLagTimeConstant = p.motor.timeConstant;

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
% The move window must cover the FULL lean-limited trapezoid: the
% profile caps rampTime at moveDuration/2, so a 10 s window forced the
% 0.524 m/s reference onto a 5 s ramp = 0.105 m/s^2 -- beyond the
% 0.008 rad lean cap, pinning every cycle at the cone edge until a
% perturbation tipped it (~3 s in every experiment).  18 s allows the
% 8.7 s ramp up + 8.7 s down at the designed 0.06 m/s^2, so the lean
% demand stays inside the cap with margin instead of saturating.
p.command.cycleMoveDuration = 18.0;
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
% Goal: cruise at 0.306 m/s, ramped trapezoidally at accelerationLimit.
% commandScale derates the planar reference, so the profile requests the
% value that leaves commandScale*command == target speed.
p.command.profileVelocityWorld = [ ...
    0.306/p.controller.fullplant.commandScale; 0];
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
% limit-cycle exciter.  But a slow filter STARVES the balance loop:
% during wheel rocking the filtered speed lags, the back-EMF term
% over-estimates the duty headroom, and delivered torque fell to ~half
% the command (measured: 0.02-0.036 N*m delivered vs +-0.0586 asked).
% A near-instantaneous constant restores torque tracking; the residual
% flicker passes into duty but the motor inductance absorbs it.
p.controller.torqueDrive.backEmfSpeedTimeConstant = 1.0e-3;
% Gyro-rate low-pass [s] ahead of state feedback: the estimator passes
% bias-corrected raw gyro, and with a noisy IMU the rate channel would
% write sigma_gyro*K_rate of torque noise straight into the command.
% 20 ms lags the ~9 rad/s tilt poles by only ~10 deg but knocks the
% broadband floor down; kept short of the ~30 ms where the added phase
% starts eating the restored damping margin.
p.controller.torqueDrive.rateFilterTimeConstant = 0.02;
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
% are reset near the LQR design (tilt x1.8 clearing the heavy-build
% static-hold floor, tilt-rate x1.0 restoring damping, yaw rate
% x5).  Velocity and
% wheel-rate columns stay zero: velocity tracking in torque mode has no
% feedforward yet, and wheel-rate feedback re-injects encoder
% quantization.
if nargin >= 1 && ~isempty(payloadMass)
    payloadForDesign = payloadMass;
else
    payloadForDesign = 0.5;
end
[p.controller.torqueLqi.stateGain, p.controller.torqueLqi.robustness] = ...
    ballbotRobustTorqueGains(p, payloadForDesign);
% Gravity-hold feedforward on the lean setpoint (see
% ballbotRobustTorqueGains for the derivation): without it the
% equilibrium tilt amplifies the commanded lean by BK/(BK-gg) ~9x and a
% 0.008 rad demand drifts to ~0.07 rad -- past the ~0.016 rad static
% cone.  leanHoldTorque maps [pitch; -roll] lean demand to the per-wheel
% hold torque that cancels gravity at that lean.
p.controller.torqueDrive.leanHoldTorque = ...
    p.controller.torqueLqi.robustness.leanHoldTorque;
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
% Lean-setpoint slew limit [rad/s]: the ball accelerates only through
% the wheel-contact friction, so a step lean command drives the chassis
% ahead of its support and relPos converts the offset into tilt runaway
% within ~0.1 s.  0.05 rad/s ramps a full 0.008 rad lean in ~160 ms,
% keeping the commanded tilt inside the rate the ball can track.
p.controller.torqueDrive.leanRateLimit = 0.05;
% Tilt-gain scheduling: tilt disturbance sensitivity scales like
% 1/(B*K_tilt - mgh/I), so the quiet low-gain design (IMU-noise
% robustness) leaves almost no rejection exactly where the pendulum
% needs it -- near the ~0.016 rad static hold cone.  Past the commanded
% lean band the tilt+rate columns stiffen smoothly to x3 over a
% tiltStiffenTilt-wide ramp, reaching near-full authority just inside
% the cone edge.  Inside the commanded band small-signal torque stays
% ~1/3 of the clamp, so gyro noise does not write into the wheels.
p.controller.torqueDrive.tiltStiffenGain = 3.0;
p.controller.torqueDrive.tiltStiffenTilt = 0.006;
% Slow tilt-error integrator [1/s scale on the tilt columns, rad*s
% state limit, 1/s leak]: the fixed proportional gain sits only ~5%
% above the 1 kg static-hold floor, so the net tilt stiffness
% B*K-gg is ~5% of nominal and any residual torque (contact friction,
% asymmetric wheel loading, relPos gravity moment m*g*dx) walks the
% tilt ~0.004 rad/s into the cone (all runs tip at ~2-4 s).  A bounded
% leaky integrator on the tilt error absorbs that constant residual and
% pulls the equilibrium back to the commanded lean; the leak keeps it
% from winding up during saturation, and the state limit caps its
% torque at ~Ki*Kt*limit -- well under the traction clamp.
p.controller.torqueDrive.tiltIntegralGain = 0.5;
p.controller.torqueDrive.tiltIntegralLimit = 0.01;
p.controller.torqueDrive.tiltIntegralLeak = 0.5;
% Soft-start torque ramp [s]: the first ~0.3 s after enable is the
% contact-settling transient (asymmetric wheel grabs kick tilt to
% ~0.014 rad, beyond the ~0.013 rad hold cone on heavy builds).  A
% ramped torque budget catches the settle progressively instead of an
% instant full-saturation kick that shoves the chassis past the cone.
p.controller.torqueDrive.startupTorqueRampTime = 0.4;
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

% =====================================================================
% Cascaded architecture (MiaPURE-style, arXiv:2304.02887): a 6-state
% LQR owns [tilt(2); tiltRate(2); ballVelocity(2)], the outer loop maps
% ball-velocity error to a commanded lean, and a bounded leaky PI on the
% ball-velocity tracking error compensates unmodeled stiction.  This
% replaces the tilt-only torque LQI whose uncontrolled relPos degree of
% freedom walked the tilt into the hold cone ~2-4 s into every move.
p.controller.cascade.enabled = true;
p.controller.cascade.velocityStateFeedback = false;
[p.controller.cascade.stateGain, p.controller.cascade.info] = ...
    ballbotCascadeGains(p, payloadForDesign);
% stateGain keeps only the physical-state columns; the lag-state
% columns live in appliedTorqueGain and feed back the internally
% tracked applied torque (delay-aware LQR term).
p.controller.cascade.stateGain = p.controller.cascade.stateGain(:, 1:6);
p.controller.cascade.appliedTorqueGain = ...
    p.controller.cascade.info.appliedTorqueGain;
p.controller.cascade.actuatorLagTimeConstant = ...
    p.controller.cascade.info.actuatorLagTimeConstant;
% Outer velocity->lean loop, same physical bounds as the torque-LQI
% outer channel.
p.controller.cascade.velocityLeanGain = 0.3;
% Command-acceleration feedforward [fraction of a_cmd/g added to the
% commanded lean]: the velocity-error path alone builds the required
% lean only after the error accumulates, so the ball overshoots ~2x the
% ramp before the correction arrives.  Feeding the command's own
% acceleration forward cuts the lag (measured: tilt stayed under
% ~0.02 rad through 8+ s vs ~0.15 rad without it).
p.controller.cascade.accelFeedforwardGain = 1.0;
% Translation-gated yaw-rate damper [N*m common-mode per rad/s]: the
% gated yaw channel is off during translation, so ball z-spin drags
% the chassis yaw (~0.1 rad per leg) and rotates the commanded push
% direction.  A common-mode damper holds heading while a translation
% command is active; at rest the same gain would pump a yaw-rate/gyro
% limit cycle, so the update code only applies it for commandSpeed>0.
p.controller.cascade.yawDampingGain = 2.0;
p.controller.cascade.leanSetpointLimit = 0.008;
p.controller.cascade.leanFadeTilt = 0.02;
p.controller.cascade.leanRateLimit = 0.05;
% Same gravity-hold feedforward as the torque path: Btilt*M = -gg*I.
p.controller.cascade.leanHoldTorque = ...
    p.controller.torqueLqi.robustness.leanHoldTorque;
% Gyro-rate LPF for the noisy-IMU small-signal requirement.
p.controller.cascade.rateFilterTimeConstant = 0.02;
% Soft-start ramp applied to disturbance paths only (FF + PI + velocity
% state feedback + lean reference), never to the tilt feedback itself.
p.controller.cascade.startupTorqueRampTime = 0.4;
% Passive settle window before the drive engages [s]: the contact grab
% transient excites an ~8 Hz rocking mode which the tilt/rate feedback
% AMPLIFIES (closed-loop kick ~0.016 rad vs ~0.002 rad passive), so zero
% torque for the window lets the contacts settle first.  The unpowered
% chassis diverges only on the ~2 s pendulum timescale, so a few hundred
% ms is safe.
p.controller.cascade.settleDelayTime = 0.35;
% Slip guard [m/s]: when the ball-rate-derived velocity and the rover
% velocity disagree by more than this, the ball is slipping/spinning on
% the ground; the velocity path freezes so it cannot pump the slip.
p.controller.cascade.slipVelocityThreshold = 0.15;
% RelPos centering [m/s^2 per m]: leans the chassis toward the rover-
% ball offset so the ball is re-centered by gravity.  Modest -- a
% nonzero relPos is the correct acceleration equilibrium; the demand
% cap keeps it a small bias next to the velocity channel.
% Lean-domain centering stays off: the lean demand can only move the
% chassis, which injects tilt disturbance faster than it re-seats the
% ball (gain sweeps 2->15 shortened hold time).  Centering lives in the
% direct torque channel below.
p.controller.cascade.positionGain = 0;
p.controller.cascade.positionDemandLimit = 0.08;
% Direct relPos -> wheel-torque PD (MiaPURE ball-angle state): a
% zero-sum wheel torque rolls the ball back under the chassis without
% going through the slow lean path.  The reaction leans the chassis and
% is absorbed by the tilt loop, so the demand is capped well inside the
% traction budget.
% The cap and damping were raised after the delivered-torque fix (near-
% instantaneous back-EMF compensation): ball-escape containment, not
% rocking noise, is the binding failure mode.  150/40/0.9 roughly
% tripled move-phase survival in sim sweeps.
p.controller.cascade.positionTorqueGain = 150.0;
p.controller.cascade.positionTorqueDamping = 40.0;
p.controller.cascade.positionTorqueLimit = 0.90;
% Wheel-rate damping [N*m per rad/s]: the tilt-only design carried
% slip-mode damping through its wheel-rate columns; the cascade needs
% it explicitly or wheel-ball slip pumps ball spin during transients.
% 0.02 matches the legacy slipDampingGain scale.
p.controller.cascade.wheelRateDamping = 0.02;
% Deadzone [rad/s] on the slip rate: encoder quantization/sampleTime
% flicker is ~3.7 rad/s; the legacy value 4.42 is measured.
p.controller.cascade.slipDeadzone = 4.42;
% Ball vertical-spin brake gain [N*m per rad/s of omegaBz], applied as
% common-mode torque (sign convention measured: +common torque gives
% -z ball accel, so braking torque = +k*omegaBz).  Fills the encoder
% deadzone where the wheel-rate channel cannot see the spin.
p.controller.cascade.ballSpinBrakeGain = 0.02;
% Ball-velocity PI: allocation pinv(Bvel) gives the minimum-norm wheel
% torque for a desired ball acceleration; the tilt loop absorbs the
% residual tilt reaction.
p.controller.cascade.ballVelocityAllocation = ...
    pinv(p.controller.cascade.info.BvelNominal);
p.controller.cascade.ballVelocityKp = 0.3;
p.controller.cascade.ballVelocityKi = 0.5;
p.controller.cascade.ballVelocityLeak = 0.5;
p.controller.cascade.ballVelocityIntegralLimit = 0.5;
% Stationary yaw gate + common-mode allocation through the measured yaw
% B row.
p.controller.cascade.yawGate.maxSpeed = 0.03;
p.controller.cascade.yawGate.maxTilt = 0.02;
p.controller.cascade.yawGate.commandThreshold = 1.0e-3;
p.controller.cascade.yawGate.rateThreshold = 0.05;
yawRow = [22.7010, 22.6692, 22.7372];   % measuredB0 yaw row (nominal)
p.controller.cascade.yawAllocation = pinv(yawRow);
p.controller.cascade.yawRateGain = 0.2;

% Identifies which builder produced this set so the rev1 model InitFcn
% can regenerate ballbotParams when a stale variant (which lacks the
% electrical actuator fields the Simscape network requires) is left in
% the workspace.
p.parameterSource = "ballbotRev1Parameters";
end
