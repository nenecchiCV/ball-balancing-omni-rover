function p = ballbotRev1Parameters(payloadMass, sampleTime, design)
%BALLBOTREV1PARAMETERS Parameters for the rev1 Simscape-actuator model.
%   p = BALLBOTREV1PARAMETERS returns the parameters for
%   ball_balancing_omni3_multibody_lqi_rev1.slx: the full-plant parameter
%   set extended with the electrical actuator quantities that the Simscape
%   motor/driver network requires.
%
%   p = BALLBOTREV1PARAMETERS(payloadMass) replaces the mounted payload
%   mass [kg] and recomputes every payload-dependent plant quantity.  The
%   legacy LQI and PI gains intentionally stay at the nominal design; the
%   torque-LQI robust gains are designed at the target payload (the hold
%   floor and the traction saturation cone do not intersect for a single
%   gain across the family, and the mounted payload is physically known).
%
%   p = BALLBOTREV1PARAMETERS(payloadMass, sampleTime) also overrides
%   the estimator/controller discrete rate [s] (default 5 ms); every
%   Ts-derived quantity (encoder flicker thresholds, IMU noise sigma,
%   discrete gain redesign via c2d) is rebuilt consistently.

if nargin < 2
    sampleTime = [];
end
if nargin < 3
    design = [];
end

% Rebuilding parameters always starts a fresh experiment: drop the
% persistent state held by the model's interpreted MATLAB Function
% blocks (estimator, controller, IMU noise stream) so the next sim
% matches a first-run result.  The InitFcn only calls this builder
% when ballbotParams is missing or stale, so pressing Run again in the
% same session keeps warm state; run `clear functions` by hand to
% force a cold start in that case.
clear functions

p = ballbotFullPlantParameters(sampleTime, design);

% Plant augmentation (ball-ground rolling resistance, wheel-shaft
% friction) is applied at runtime so the .slx stays in R2024b format.
% InitFcn calls this function at every simulation start, so augmenting
% here covers the GUI-run path too; guarded for param builds with no
% model loaded.  Runs at the END of this builder so flag fields the
% augmentation consumes are already populated.

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
% Recovery-mode traction allowance [N*m]: a tilted-start righting
% maneuver needs ~0.44 N*m of moment about the ball centre at 10 deg,
% but the nominal cap above only supplies ~0.49 N*m ideal (which
% evaporates once the uphill wheel unloads).  Doubling the nominal cap
% lets recovery reach ~0.98 N*m -- measured to right 10 deg starts in
% ~0.4 s without exciting wheel slip.  Raising it further (0.25+)
% overdrives the wheels into slip and makes recovery WORSE; the
% validated working band is roughly [0.145, 0.25).  Applies to mode=2
% only, so normal-operation slip budgeting is unchanged.
p.controller.recoveryWheelTorqueLimit = min( ...
    p.driver.continuousTorqueLimit, ...
    2.0*p.controller.tractionLimitedWheelTorque);

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
% into the same first-order model.  Retained as documentation of the
% lag the robust torque-LQI design absorbs.
% removed (experiment history lives in README and git log).

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
% Move window sized to the retargeted 0.10 m/s-class profile: at
% accelerationLimit the 0.167 m/s reference ramps in ~5.6 s, so 12 s
% holds cruise ~0.8 s plus the symmetric ramp-down.
p.command.cycleMoveDuration = 12.0;
% Yaw control is fully separated from translation: the yaw-rate request
% is issued as a trapezoid inside the stop window, starting cycleYawDelay
% seconds after motion stops.  The stationary yaw gate in the controller
% opens only while command speed is zero, so the profile must guarantee
% the yaw window lies inside a stop phase with margin on both sides.
p.command.cycleStopDuration = 4.0;
% Measured safe in-cycle yaw on the v12 plant (obs-off, 49.5 s cycle):
% 0.2 rad/s x 1.0 s trapezoid (~0.1 rad of rotation) at the START of the
% stop window, thinned to every other stop phase via cycleYawEvery=2,
% completes the full run (maxTilt 1.30 deg).  Denser or larger bursts
% all fail: 0.1 rad every cycle dies at cycle 3 (t=46-49 s), >=0.15 rad
% dies after a single burst (t=23-27 s), and even a 6 s stop does not
% save every-cycle 0.1 rad (t=33 s).  Residual ball z-spin plus the
% normal-force asymmetry the burst leaves behind is what kills the next
% deceleration corner, so the burst budget is ~0.1 rad per full
% move/stop period, not per stop window.
p.command.cycleYawDelay = 0;
p.command.cycleYawDuration = 1.0;
% Thins the yaw window to every cycleYawEvery-th stop phase so a full
% move/stop period elapses between bursts (residual ball z-spin decays
% before the next one).  1 = every stop window, 0 disables thinning.
p.command.cycleYawEvery = 2;
p.command.cycleCount = 3;
% The holdable lean cone is ~0.023 rad of tilt at the full traction
% budget, and propulsion torque spends part of that budget during the
% ramp.  Commanded acceleration is therefore capped well inside the
% cone: 0.06 m/s^2 ~ lean 0.006 rad, about a quarter of the limit.
% Deceleration is the critical phase: braking transfers chassis momentum
% to the ball, which surges ~+0.05 m/s and dies at the ~0.2 m/s contact
% wall.  0.03 keeps the surge recoverable at the retargeted speed.
p.command.accelerationLimit = 0.03;
% Braking is asymmetric with acceleration on the real machine too: the
% braking torque margin is thinner (decel drives the chassis pitch-back
% that kicks the ball forward), so the ramp-down gets its own gentler
% limit.  ballbotRev1CommandProfile applies it to the move-window tail.
% Measured at the mu_est=1.0 traction budget: 0.02 dies in the first
% stop window (~15 s, ball still rolling 0.24 m/s rolls out sideways),
% 0.01 roughly doubles survival (~39-41 s, mid cycle 3).  0.005 is
% worse again -- too-slow decel enters the stop phase with even more
% residual ball speed, so do not simply keep reducing it.
p.command.decelerationLimit = 0.01;
% Bounded-yaw pulse length [s] for the non-cycle command path: the
% plant loses a wheel ~0.4-0.8 rad into any sustained common-mode yaw
% (the wheel torque pumps the ball's free z-spin), so scenarios cap the
% rotation instead of holding a rate indefinitely.  0 disables.
p.command.yawPulseDuration = 0;

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
% Cruise target: 0.12 m/s -- the demonstrated envelope is bounded below
% by the encoder stillness blind band (ball speeds under ~0.03 m/s fall
% inside the quantization flicker zone) and above by the ~0.2 m/s
% contact wall where a wheel unloads.  The requested 0.306 m/s lies
% beyond the wall.  NOTE: commandScale only applies in the legacy
% fullplant hierarchy path; the torque-LQI path commands the raw
% profile value, so profileVelocityWorld IS the physical setpoint.
p.command.profileVelocityWorld = [0.12; 0];
% Cycle stop-window yaw trapezoid peak.  Sustained in-cycle yaw is
% outside the plant envelope (see the cycleYawDuration comment); the
% verified-surviving burst is 0.2 rad/s x 1.0 s (~0.1 rad) every other
% stop window.
p.command.profileYawRate = 0.2;

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
% Wheel-rate LPF for the slip damper and propulsion servo only (the LQI
% state still sees the raw rate): quantization flicker is ~3.7 rad/s
% per sample versus a ~5-6 rad/s cruise rolling rate, so the raw rate
% is mostly noise and thrashes these channels to the torque clamp.
p.controller.torqueDrive.wheelRateFilterTimeConstant = 0.03;
% Deadzone for the FILTERED wheel-rate channels (slip damper, propulsion
% servo): with the flicker low-passed away, the raw flicker-sized
% slipDeadzone (~4.4 rad/s) would hide real rate errors -- at cruise a
% 4 rad/s error IS a ~0.13 m/s ball-velocity overrun that these channels
% exist to correct.  ~1 rad/s keeps the residual filtered flicker out
% while passing genuine slip/overshoot.
p.controller.torqueDrive.filteredSlipDeadzone = 1.0;
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
% Yaw command gate [SI]: the yaw-rate feedback column is fed the
% yaw-rate state error when a yaw rate above commandThreshold is
% commanded AND the rover is either near rest (maxSpeed/maxTilt window,
% commandSpeed == 0 -- the cycle profile's stop-window yaw) or under a
% nonzero commanded translation (moveYaw -- the coupled (v, yaw)
% limiter bounds the shared wheel-rate budget).  Below the threshold
% the yaw state is zeroed and the passive yaw damper owns the channel.
p.controller.torqueDrive.yawGate.maxSpeed = 0.03;
p.controller.torqueDrive.yawGate.maxTilt = 0.02;
p.controller.torqueDrive.yawGate.commandThreshold = 1.0e-3;
p.controller.torqueLqi.integralGain = zeros(3, 2);
% Velocity-state columns for the torque LQI: ballbotVelocityTorqueGains
% solves the 10-state plant [v; tilt; rate; yaw; wheels] whose dv/dt =
% g*tilt + forceScale*W'*tau/mEff, i.e. wheel torque produces a DIRECT
% tangential contact force on the ball (the measured forceScale=0.5
% coupling).  The feedback then maps the velocity error straight to
% wheel torque -- drift braking uses the same symmetric channel as
% driving, instead of pumping the lean->gravity outer loop.  The
% controller feeds state(1:2) = ballVelocityBody - commandVelocityBody
% only during elevated-authority settle (gated in the controller: in
% normal driving the force channel traction-drives the coasting ball).
% velocityStateQv ~1000 gives ~0.34 N*m per m/s per wheel: a 0.2 m/s
% drift draws ~0.07 N*m/wheel, inside the traction budget.
p.controller.torqueDrive.velocityStateQv = 1000;
[velocityColumns, velocityGainDesign] = ballbotVelocityTorqueGains( ...
    p, payloadForDesign, p.controller.torqueDrive.velocityStateQv);
p.controller.torqueLqi.stateGain(:, 1:2) = velocityColumns;
p.controller.torqueLqi.velocityGainDesign = velocityGainDesign;
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
% torque-limited hold cone (~0.023 rad measured at the mu_est=0.75
% clamp; the cone scales with the traction budget, widening to ~0.031
% rad at mu_est=1.0): a lean demand near the cone edge tips at t ~1 s
% because any tilt overshoot past the cone is unrecoverable (gravity
% pole ~6.4 rad/s, ~150 ms e-folding).  0.016 rad is about half the
% widened cone and lifts the gravity-drive speed cap that throttled the
% ball to ~0.13 m/s at the 0.008 rad setting.
p.controller.torqueDrive.velocityLeanGain = 0.3;
% Saturation-aware re-derivation (limit-cycle redesign): the commanded
% lean band is now derived from the tilt-column gain, not set
% independently.  With K_tilt ~6.5 N*m/rad (R=4000 design) a 0.016 rad
% edge demanded 0.104 N*m -- 133% of the 0.078 clamp -- so the loop ran
% bang-bang precisely inside the cone it was supposed to hold.  Capping
% the band at 0.009 rad bounds the edge demand at ~75% of the clamp;
% the residual tracking lean for a 0.03 m/s^2 ramp is only ~0.003 rad,
% so translation authority is unaffected.
p.controller.torqueDrive.leanSetpointLimit = 0.009;
% Command-acceleration feedforward gain on the lean demand: the ramped
% command needs lean ~a_cmd/g (0.003 rad at 0.03 m/s^2), far below the
% P-error saturation cap, so feedforward carries the nominal lean and
% the error term only trims tracking lag.  1.0 is the model value.
p.controller.torqueDrive.accelFeedforwardGain = 1.0;
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
% tiltStiffenGain halved (4.0 -> 2.0) in the same redesign: at x4 the
% boosted tilt column (~26 N*m/rad) railed the clamp for any tilt only
% 0.003 rad past the lean band, which IS the bang-bang limit cycle
% observed at ~0.13 m/s.  x2 still stiffens past the band but keeps the
% maximum tilt-channel demand at a recoverable level.
p.controller.torqueDrive.tiltStiffenGain = 2.0;
p.controller.torqueDrive.tiltStiffenTilt = 0.006;
% Observer-based wheel-rate estimate (encoder-quantization fix for the
% limit cycle): the raw 341.2-PPR backward difference injects +-3.7
% rad/s flicker per 5 ms sample, which the tilt-rate column converts to
% >2 N*m of torque noise -- the dominant saturation pump on the hold
% and small-signal trajectory.  Model-aided filter: each control step
% the estimate advances by the measured wheel-drive dynamics (last
% torque command minus the compliance drag (wheelRate - kinematicRate))
% and corrects toward the measured rate with gain observerGain.  On a
% synthetic small-signal regime the observer cuts rate error ~6x vs
% backward difference and ~3x vs the 30 ms lag filter it replaces.
% 0 keeps the legacy lag-filtered backward difference.
%
% DISABLED by default (measured on v12 plant): driveGain=306 predicts
% the free-wheel rate response, but under ball contact the loaded
% response is ~100x slower, so the estimate inflates well above the
% measured rate during any sustained torque.  The inflated rate kills
% the propulsion error (rateError -> 0) AND raises slipRate into the
% damper's active band (phantom braking): commanded 0.306 m/s stalled
% at 0.017 m/s median vs 0.044 with the observer off; driveGain 30/5
% recover only part of it.  The cycle still completes 49.5 s without
% the observer (tilt 0.84 deg vs 0.38 deg), so the flicker pump it was
% built to suppress is not fatal at current operating points.
p.controller.torqueDrive.wheelRateObserverEnabled = false;
p.controller.torqueDrive.wheelRateObserverGain = 0.15;
p.controller.torqueDrive.wheelRateObserverDriveGain = 306;
p.controller.torqueDrive.wheelRateObserverCompliance = 0.134;
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
p.controller.torqueDrive.tiltIntegralGain = 1.0;
p.controller.torqueDrive.tiltIntegralLimit = 0.03;
p.controller.torqueDrive.tiltIntegralLeak = 0.5;
% Soft-start torque ramp [s]: the first ~0.3 s after enable is the
% contact-settling transient (asymmetric wheel grabs kick tilt to
% ~0.014 rad, beyond the ~0.013 rad hold cone on heavy builds).  A
% ramped torque budget catches the settle progressively instead of an
% instant full-saturation kick that shoves the chassis past the cone.
p.controller.torqueDrive.startupTorqueRampTime = 0.4;
% Propulsion: P servo tracking the kinematic wheel rate W*v_cmd, in
% N*m per rad/s of wheel-rate error.  The tilt loop handles attitude;
% this term only drives the wheel/ball rolling state toward the command.
% 0.02 gives real braking authority during deceleration: the same servo
% is what drags a coasting ball down through the contact, and at 0.005
% the brake force was a fraction of the momentum-transfer surge.
p.controller.torqueDrive.propulsionGain = 0.02;
% Propulsion torque share cap [N*m]: without a bound the initial rate
% error (~20 rad/s for a 0.5 m/s command) saturates one wheel's duty to
% ~0.95 while the tilt channel starves.  0.045 (~75% of the traction
% limit) gives the braking direction enough bite to hold the ball below
% the ~0.2 m/s contact wall during decel, while leaving headroom for
% attitude correction.
p.controller.torqueDrive.propulsionTorqueLimit = 0.045;
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
% NOTE: with rolling resistance this term was re-tested and REJECTED --
% it overdrives the wheels while the ball lags and the reaction slides
% the chassis backwards through the accel phase (measured: x drifted
% -0.15 m before the decel corner).  Ships at 0.
% removed (experiment history lives in README and git log).
% Follow-through braking [dimensionless, <1]: whenever the commanded
% speed drops below this fraction of the measured ball speed --
% deceleration overrun or a stop command on a still-rolling ball -- the
% command follows the ball instead.  Without it the wheel servo locks
% against the rolling ball and the contact drag kicks the roll channel
% over (first stop window fell at ~14 s every run).  With 0.8 the ball
% bleeds off smoothly (vB 0.24 -> 0.05 over the stop window) and the
% stop phase survives; the residual failure moves into the next move
% leg.  0 disables.
p.controller.torqueDrive.stopFollowGain = 0.75;
% Post-recovery settle follow gain: replaces stopFollowGain while
% elevatedAuthority is held (mode 1 after a recovery).  The 0.75 brake
% fraction demands torque AGAINST the residual ball roll; in the weak
% -y wheel-sharing direction that demand pumps the ball faster than it
% decays (+2.5 rad/s^2 measured on a righted 10 deg roll start).
% ~1.0 is a pure chase -- wheels roll WITH the ball at its own rate so
% the contact needs no force and rolling resistance sheds the drift.
% Values >1 drag the ball back under the chassis actively.
p.controller.torqueDrive.settleFollowGain = 1.0;
% Settle lean damping [s]: derivative term on the velocity outer loop,
% applied only during elevated-authority settle.  Also doubles as the
% IMU accel-bias compensation: the tilt estimate reads real+a/g during
% drift (measured est-real ~0.016 rad at ~0.1 m/s^2), so subtracting
% the true encoder-derived acceleration puts the demanded setpoint in
% the right place in estimate space.  ~1.0 holds the 10 deg roll
% upright >20 s (vs ~8 s at 0.05); it does not stop the coasting drift.
p.controller.torqueDrive.settleLeanDamping = 1.0;
% Settle-only velocity-lean gain [m/s^2 per m/s]: replaces
% velocityLeanGain during elevated-authority settle.  The drift brake
% needs a lean demand larger than the IMU accel bias + attitude droop;
% the normal-drive value (0.3) cannot push the real lean back through
% zero against them.  Kept separate so normal move legs keep the
% validated gain.
p.controller.torqueDrive.settleLeanGain = 1.0;
% (settleRampTime / settleRelPosGain / settlePumpGain /
% settleYawDampGain experiments removed: a ramped or slipped chase
% command pumps the lean->runaway cycle faster than pure chase; the
% relPos estimate carries an ~11 cm bias during drift so the
% re-centring pull commanded phantom motion; the one-way wheel-accel
% damper turned encoder flicker into a unidirectional drag; and
% damping the yaw column did not reach the shimmy.)
% Eccentricity guard: entering a move leg with the ball ahead of the
% chassis (relPos < 0 along the command direction) is the observed
% cycle-fall trigger -- the accel lean kicks a wheel off the
% off-centre ball.  Scales accelerationDemand by
% max(floor, 1 - gain*max(-relPos.u - margin, 0)).  gain 12 /
% margin 0 / floor 0.1 extends cycle tFall 33.6 -> 43.8 s at the
% default seed (sweep 8/10/12/13/14/15/18 -> 35/39/44/40/35/41/36 s)
% with hold/stepX/yaw unchanged; at 2 ms the guard is mildly harmful
% (39.0 s vs ~52 s unguarded) because the fast loop catches the onset
% itself.  0 disables.
p.controller.torqueDrive.accelRelPosGuardGain = 12;
p.controller.torqueDrive.accelRelPosMargin = 0;
p.controller.torqueDrive.accelRelPosGuardFloor = 0.1;
% Static-lean (lateral CG offset) adaptation: with an offset payload
% the equilibrium attitude is tilted; holding tilt=0 leaves the offset
% gravity torque unopposed and the assembly drifts toward the offset
% (rest torque is ~zero -- the wedge sits on contact friction, so the
% residual signature is the drift velocity, not a holding torque).
% At zero command the low-passed drift velocity integrates slowly into
% a lean setpoint bias that tilts the equilibrium AWAY from the drift;
% the update self-stops at the true equilibrium.  Safety structure:
% learning only arms on a rest period that ENTERS from stillness
% (lagging estV < 0.02 on the rest-entry edge -- a cycle stop window
% enters already rolling, so deceleration-corner creep can never be
% mistaken for an offset), the drift must be accelerating but below a
% rate cap (a coast is not an offset), and the bias only injects at
% zero command plus leaks while commanded.  Verified: 10/15/20 mm rod
% offsets all hold (transients 1.0/3.9/4.1 deg), zero-offset hold and
% the move/stop cycle are bit-identical to baseline.
p.controller.torqueDrive.staticLeanAdaptGain = 0.5;
p.controller.torqueDrive.staticLeanAdaptLimit = 0.10;
p.controller.torqueDrive.staticLeanAdaptTimeConstant = 0.30;
p.controller.torqueDrive.staticLeanAdaptDeadband = 0.02;
p.controller.torqueDrive.staticLeanAdaptGate = 0.30;
p.controller.torqueDrive.staticLeanAdaptMaxRate = 1.0;
p.controller.torqueDrive.staticLeanAdaptLeak = 0.3;
% Ball vertical-spin brake [N*m per rad/s of omegaBall_z]: translation
% pumps ball z-spin through the omni kinematics and through the
% common-mode residual of saturated torque patterns; the accumulated
% spin is what walked the cruise tilt into the cone (~0.01 -> 0.02 rad
% per move leg) and unloads a wheel past ~0.2 m/s.  0.06 removed the
% cruise tilt walk at the mu_est=0.75 budget; with the raised mu_est=1.0
% traction budget, 0.12 damps the spin residual faster without starving
% the balance channels (the clamp is ~33% wider, so the brake's share
% is affordable).
p.controller.torqueDrive.ballYawGain = 0.12;
% Brake torque cap [N*m]: the spin brake is a common-mode term, so at
% |oBz| ~0.5 rad/s the uncapped request (~0.06 N*m) would eat most of
% the ~0.078 N*m traction clamp and cancel the zero-sum tilt pattern on
% the wheels where they disagree -- measured: the first stop transition
% starved the roll channel this way.  0.025 leaves the brake effective
% against spin buildup while keeping ~2/3 of the budget for attitude.
p.controller.torqueDrive.ballYawTorqueLimit = 0.025;
% Passive yaw damper during translation [N*m common-mode per rad/s of
% yaw rate]: the gated yaw channel is off while a translation command
% is active and ball z-spin drags the chassis yaw, rotating the push
% direction sideways.  Gated to commandSpeed > 0 so standstill sees no
% yaw/gyro-noise limit cycle.  0.02 was far too weak: at cruise the
% chassis yawed ~0.2 rad/s (~90 deg per 10 s), sweeping the wheel loads
% periodically until a wheel unloaded -- the dominant low-speed cruise
% failure.  0.10-0.30 all held cruise 15 s at <=0.011 rad tilt; 0.30
% keeps the drift fully damped without measurable budget cost.
p.controller.torqueDrive.yawDampingGain = 0.30;

% Mounted-IMU realism (LSM6DSV16X on X-NUCLEO-IKS4A1, sampled at the
% 200 Hz estimator rate).  ballbotImuFromJoint applies these when
% enabled: datasheet noise densities (gyro 2.8 mdps/sqrt(Hz), accel
% 60 ug/sqrt(Hz)) converted to per-sample sigma by ND/sqrt(2*Ts), and
% 16-bit word quantization at the selected full scales (+/-2000 dps,
% +/-4 g).  Bias vectors carry the constant zero-rate offsets; the yaw
% bias test injection remains on p.simulation.imuYawBias and adds on
% top.  vibration* injects a sinusoidal specific-force disturbance for
% the vibration scenario (C-12); amplitude 0 disables.  noiseSeed fixes
% the noise stream so runs are reproducible (reset via clear functions,
% which the scratch scripts already do).
p.imu.enabled = true;
p.imu.noiseSeed = 20261015;
p.imu.gyroNoiseDensity = 2.8e-3*pi/180;          % rad/s/sqrt(Hz)
p.imu.accelNoiseDensity = 60e-6*p.gravity;       % m/s^2/sqrt(Hz)
p.imu.gyroQuantum = 4000*(pi/180)/65536;         % +/-2000 dps, 16 bit
p.imu.accelQuantum = 8*p.gravity/65536;          % +/-4 g, 16 bit
p.imu.gyroBias = [0; 0; 0];
p.imu.accelBias = [0; 0; 0];
p.imu.vibrationAmplitude = 0;                    % m/s^2
p.imu.vibrationFrequencyHz = 37;                 % < Nyquist of 200 Hz
p.imu.vibrationAxis = [1; 0; 0];
% Gearmotor output-shaft friction (FIT0521, gear ratio 34.02): the
% datasheet's no-load current (0.13 A) is the total loss current, of
% which roughly half is speed-independent gear stiction.  The DC Motor
% block's lam (viscous) alone already equals the no-load loss at no-load
% speed, so the stiction share is carved out: lam is halved and an
% Ideal Torque Source applies a smooth-Coulomb torque at the output
% shaft in ballbotRev1PlantAugment (runtime; keeps the .slx at R2024b).
p.motor.viscousShare = 0.5;
p.motor.outputCoulombTorque = (1 - p.motor.viscousShare)* ...
    p.motor.torqueConstant*p.motor.noLoadCurrent;
p.motor.outputBreakawayTorque = 1.2*p.motor.outputCoulombTorque;
p.motor.outputShaftFrictionEnabled = true;

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
% Controller path selection.  The torque-input LQI (torqueDrive) is the
% default: with the filtered-stillness estimator and velocity drain it
% holds machine-quiet tilt (~0.007 rad over 5 s, payload 0.5/1.0 kg
% included).  The optional cascade (MiaPURE-style, arXiv:2304.02887)
% and ballforce alternatives measured worse on every axis -- noisier
% hold, earlier cone-edge runaway, payload falls -- so both paths were
% removed (experiment history lives in README and git log).

% Identifies which builder produced this set so the rev1 model InitFcn
% can regenerate ballbotParams when a stale variant (which lacks the
% electrical actuator fields the Simscape network requires) is left in
% the workspace.
p.parameterSource = "ballbotRev1Parameters";

mdl = 'ball_balancing_omni3_multibody_lqi_rev1';
runtimeApplied = false;
if bdIsLoaded(mdl)
    % The R2026a part handler can fire the model PreLoadFcn (which calls
    % this builder) before subsystem parts resolve, so find_system inside
    % the augment may fail mid-load.  The failure must not abort the
    % load; callers re-apply the augmentation once the model is up.
    ok = true;
    try
        ballbotRev1PlantAugment(mdl, p);
    catch
        ok = false;
    end
    % Same for the runtime solver/driver settings: applying them on the
    % parameter-build path keeps an interactive Run (open the .slx, press
    % Run) on the fast configuration instead of the .slx's stored
    % conservative values (MaxStep 2e-4, freewheel-mode bridge switching,
    % ~5700 solver exceptions per 2 s).
    try
        ballbotRev1SimConfig(mdl);
    catch
        ok = false;
    end
    if ok
        runtimeApplied = true;
        % The model InitFcn only rebuilds ballbotParams when it is stale,
        % so cache misses like the PreLoadFcn path would leave later sim
        % starts without a chance to (re)apply the runtime pieces.  Add a
        % one-time block to the InitFcn itself (runtime set_param only --
        % the .slx on disk stays in R2024b format): every sim start then
        % re-applies the solver/driver/plant settings and resets the
        % interpreted MATLAB Function blocks' persistent state, giving a
        % deterministic first-run result for interactive Runs too.
        try
            init = get_param(mdl, 'InitFcn');
            tag = 'ballbotRev1RuntimeApply';
            if ~contains(init, tag)
                set_param(mdl, 'InitFcn', sprintf([ ...
                    '%s\n%% %s: re-apply runtime solver/driver/plant ' ...
                    'settings on every sim start.\n' ...
                    'clear functions;\n' ...
                    'ballbotRev1SimConfig;\n' ...
                    'if exist(''ballbotParams'',''var''), ' ...
                    'ballbotRev1PlantAugment(bdroot, ballbotParams); ' ...
                    'end'], init, tag));
            end
        catch
        end
    end
end
if ~runtimeApplied
    % Params built before the model could be fully configured (model not
    % yet loaded, or the PreLoadFcn fired mid-load): stamp the set so the
    % model InitFcn treats it as stale and rebuilds at sim start, when
    % every block is resolvable.
    p.parameterSource = "ballbotRev1Parameters-pending";
end
end
