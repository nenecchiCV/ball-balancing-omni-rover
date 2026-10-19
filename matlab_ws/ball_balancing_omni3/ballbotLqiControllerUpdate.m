function output = ballbotLqiControllerUpdate( ...
    estimate, command, previousVelocityIntegral, enable, ...
    previousYawBiasReady, biasDiagnostics, wheelRate, p)
%BALLBOTLQICONTROLLERUPDATE Ten-state MIMO LQI wheel-speed controller.

if isfield(p.controller, "fullplant") && ...
        isfield(p.controller.fullplant, "enabled") && ...
        p.controller.fullplant.enabled
    output = ballbotFullPlantHierarchyUpdate(estimate, command, ...
        previousVelocityIntegral, enable, previousYawBiasReady, ...
        biasDiagnostics, wheelRate, p);
    return
end

roll = estimate(1);
pitch = estimate(2);
yaw = estimate(3);
bodyRate = estimate(4:6);
velocityWorld = estimate(7:8);
tiltMagnitude = hypot(roll, pitch);

yawBiasReady = ballbotYawBiasStartupGuard(previousYawBiasReady ~= 0, ...
    bodyRate(3), biasDiagnostics(2) ~= 0, p);
torqueDrive = isfield(p.controller, "torqueDrive") && ...
    isfield(p.controller.torqueDrive, "enabled") && ...
    p.controller.torqueDrive.enabled;

% Recovery hysteresis: exiting to mode 1 the instant tilt dips below
% recoveryTilt hands a still-moving assembly back to the derated
% traction budget, and the post-righting drift re-tilts it (observed:
% 10 deg start righted by ~0.5 s, residual ball motion then ran the
% rover off the ball and past 0.5 rad by ~4.8 s).  Stay in recovery
% until the smaller exit threshold is crossed.
persistent prevMode
if isempty(prevMode)
    prevMode = uint8(0);
end
if ~enable || tiltMagnitude >= p.controller.fallenTilt
    mode = uint8(0);
elseif tiltMagnitude >= p.controller.recoveryTilt || ...
        (prevMode == 2 && tiltMagnitude >= p.controller.recoveryExitTilt)
    mode = uint8(2);
else
    mode = uint8(1);
end
prevMode = mode;

% Elevated traction authority: armed by entering mode 2 and kept while
% the wheels still move, so the mode-1 controller that takes over after
% righting inherits the recovery traction budget for arresting the
% residual ball roll.  Without it the derated cap lets the assembly
% drift until it re-tilts (the measured 10 deg-start failure).  Cleared
% by filtered wheel stillness -- the raw encoder flicker (~3.7 rad/s
% quantized) never crosses a small threshold, so a dedicated one-pole
% filter witnesses stillness here regardless of the observer config.
persistent elevatedAuthority settleWheelRate settleKinPrev settleKinAccel
if isempty(elevatedAuthority) || ~enable || mode == 0
    elevatedAuthority = false;
    settleWheelRate = zeros(3, 1);
    settleKinPrev = zeros(2, 1);
    settleKinAccel = zeros(2, 1);
end
if isempty(settleWheelRate)
    settleWheelRate = wheelRate(:);
end
if isempty(settleKinPrev)
    settleKinPrev = zeros(2, 1);
end
if isempty(settleKinAccel)
    settleKinAccel = zeros(2, 1);
end
% ~0.25 s one-pole: slow enough that the 3.7 rad/s encoder flicker
% averages out, fast enough to track the drift decay.
settleBlend = p.controller.sampleTime/(0.25 + p.controller.sampleTime);
settleWheelRate = settleWheelRate + ...
    settleBlend*(wheelRate(:) - settleWheelRate);
clearRate = 1.5;
if isfield(p.controller, "recoveryAuthorityClearRate")
    clearRate = p.controller.recoveryAuthorityClearRate;
end
if mode == 2
    elevatedAuthority = true;
    settleKinPrev = zeros(2, 1);
    settleKinAccel = zeros(2, 1);
elseif mode == 1 && elevatedAuthority && ...
        max(abs(settleWheelRate)) < clearRate
    elevatedAuthority = false;
end

rotationBodyFromWorld = [cos(yaw), sin(yaw); ...
    -sin(yaw), cos(yaw)];
velocityBody = rotationBodyFromWorld*velocityWorld;
commandVelocityBody = rotationBodyFromWorld*command(1:2);
commandSpeed = norm(commandVelocityBody);
% Coupled (v, yawRate) feasibility limiter: every wheel's rate demand
% is |v|*sin(lat)/Rw + Rch*|yawRate|/Rw (omni kinematics map both terms
% identically onto all three axles), so an over-budget pair must be
% scaled jointly -- clipping v alone leaves the yaw demand over budget.
if isfield(p.controller, "coupledCommandLimitEnabled") && ...
        p.controller.coupledCommandLimitEnabled
    wheelRateDemand = sin(p.wheel.contactLatitude)*commandSpeed + ...
        p.rover.chassisRadius*abs(command(3));
    wheelRateBudget = p.wheel.radius*p.motor.speedCommandLimit;
    if wheelRateDemand > wheelRateBudget
        coupledScale = wheelRateBudget/wheelRateDemand;
        commandVelocityBody = coupledScale*commandVelocityBody;
        command(3) = coupledScale*command(3);
        commandSpeed = norm(commandVelocityBody);
    end
end
if commandSpeed > p.controller.maxSpeed
    commandVelocityBody = commandVelocityBody* ...
        (p.controller.maxSpeed/commandSpeed);
end
% Follow-through braking: whenever the commanded speed is below a
% fraction of the measured ball speed (deceleration overrun, or a stop
% command on a still-rolling ball), follow the ball instead.  Locking
% the wheel-rate servo at the slower command fights the rolling ball --
% the contact drag kicks the chassis and diverges the roll channel --
% and a saturating velocity error parks the lean at its cap.  Commanding
% followGain*vBall makes the velocity error (followGain-1)*vBall: a
% speed-proportional gravity brake that decays smoothly as the ball
% sheds momentum, while the propulsion servo keeps rolling WITH the
% ball.
ballVelocityBody = rotationBodyFromWorld* ...
    [p.ball.radius*estimate(10); -p.ball.radius*estimate(9)];
% Velocity reference for the state feedback: the USER command before
% the follow-through chase rewrite.  The chase exists to keep the
% wheel servo rolling with the ball (no wheel-lock brake); the
% velocity-state gain, however, must see the true tracking error --
% during settle the rewritten command equals ball speed and would zero
% the error that drives the direct-force braking.
commandVelocityBodyUser = commandVelocityBody;
% Post-recovery settle witnesses: the estimator's ball-velocity channel
% runs hot during uncommanded drift (est ~13 rad/s vs ~1.8 rad/s actual
% in the sf100 run), so while elevatedAuthority holds the drift is read
% off the encoders -- rolling contact makes the filtered wheel rate the
% honest measure of the ball's surface speed.  Settle-active mode 1
% therefore swaps ballVelocityBody AND the chase target to the encoder
% rate (settleFollowGain ~1.0: follow the drift, let rolling
% resistance shed it) -- a biased-estimate brake here is the measured
% pump that re-tilts the chassis.
settleActive = mode == 1 && elevatedAuthority && ...
    isfield(p.controller.torqueDrive, "settleFollowGain") && ...
    p.controller.torqueDrive.settleFollowGain > 0;
settleWitness = mode == 1 && elevatedAuthority;
if settleWitness
    settleKinRate = pinv(p.controller.lqi. ...
        wheelSpeedFromPlanarVelocity)*settleWheelRate;
    % Filtered finite difference of the drift rate: the derivative of
    % a 0.25 s-filtered signal stays clean enough to damp the
    % velocity-loop oscillation the P-only lean demand cannot.
    rawKinAccel = (settleKinRate(1:2) - settleKinPrev)/ ...
        p.controller.sampleTime;
    settleKinPrev = settleKinRate(1:2);
    settleKinAccel = settleKinAccel + settleBlend* ...
        (rawKinAccel - settleKinAccel);
    ballVelocityBody = settleKinRate(1:2);
    % Rover ground speed rides the same drift; estimate(7:8) runs ~3x
    % hot here, so the slip-damper's kinematic reference and the
    % stationary gate also read the encoder-derived rate.
    velocityBody = settleKinRate(1:2);
    chaseBody = ballVelocityBody;
else
    chaseBody = ballVelocityBody;
end
if torqueDrive && isfield(p.controller.torqueDrive, "stopFollowGain") ...
        && p.controller.torqueDrive.stopFollowGain > 0
    followGain = p.controller.torqueDrive.stopFollowGain;
    if settleActive
        followGain = p.controller.torqueDrive.settleFollowGain;
    end
    chaseSpeed = norm(chaseBody);
    if commandSpeed < followGain*chaseSpeed
        commandVelocityBody = followGain*chaseBody;
        commandSpeed = norm(commandVelocityBody);
    end
end
% (Decaying chase-ramp and proportional-slip settle brakes were tried
% and rejected: every deceleration demand through this loop pumps the
% lean->runaway cycle and the drift re-tilts the chassis sooner than
% pure chase.)

if mode == 1
    % Integrate the BALL velocity error, matching the lean channel: the
    % command describes the assembly's translation and the ball is the
    % load-bearing element.  Rover velocity leads the ball during any
    % acceleration, so integrating rover error winds the integrator with
    % the transient lead every manoeuvre.
    velocityError = commandVelocityBodyUser - ballVelocityBody;
    candidateIntegral = previousVelocityIntegral + ...
        p.controller.sampleTime*velocityError;
    integralLimit = p.controller.velocityIntegralLimit;
    candidateIntegral = min(max(candidateIntegral, -integralLimit), ...
        integralLimit);
else
    candidateIntegral = zeros(2, 1);
end

state = [velocityBody; pitch; -roll; bodyRate(2); -bodyRate(1); ...
    bodyRate(3); wheelRate(:)];
yawRateCommand = min(max(command(3), -p.controller.maxYawRate), ...
    p.controller.maxYawRate);
if ~yawBiasReady && abs(command(3)) <= ...
        p.controller.yawBiasCommandBypassThreshold
    yawRateCommand = 0;
end
if torqueDrive
    % Torque-input LQI: the output is wheel torque [N*m], so the inner
    % wheel-speed servo disappears and tilt feedback acts at the
    % electrical time scale.  Yaw has no kinematic feedforward here; a
    % steady yaw rate needs zero torque, so the reference enters only
    % through the yaw-rate state error.
    lqi = p.controller.torqueLqi;
    debugCapture = isfield(p.controller.torqueDrive, "debugCapture") && ...
        p.controller.torqueDrive.debugCapture;
    if debugCapture
        captureInputs = [estimate(:); command(:); ...
            previousVelocityIntegral(:); enable; previousYawBiasReady; ...
            biasDiagnostics(:); wheelRate(:)];
    end
    if isfield(p.controller.torqueDrive, "openLoopTorque")
        % Plant-identification hook: bypass all feedback and apply a
        % fixed wheel-torque vector.
        output = [p.controller.torqueDrive.openLoopTorque(:); ...
            zeros(2, 1); double(mode); double(yawBiasReady)];
        return
    end
    % Yaw/planar separation: common-mode torque (the only yaw actuator)
    % pumps the ball's free spin mode, and that exchange is what tipped
    % the plant at ~5 s in the truth-feedback comparison.  The yaw-rate
    % channel is therefore enabled ONLY while a yaw command is actually
    % requested.  Commanded yaw during translation (moveYaw) shares the
    % wheel-rate budget with the planar command through the coupled
    % limiter upstream; the ballYawGain spin brake below absorbs the
    % spin the common-mode exchange pumps.  Damping residual yaw rate
    % at standstill also pumps the ball spin (measured: oBall_z ~21
    % rad/s inside 0.15 s while parked at zero command), so the gate
    % keys on the command alone; uncommanded yaw drift is left to
    % friction.
    % Gyro-rate low-pass for the state feedback path.  The estimator
    % outputs bias-corrected raw gyro, so noisy IMUs (the planned cheap
    % unit) would inject torque noise proportional to the rate gain.
    % A ~20 ms first-order lag costs ~10 deg of phase at the ~9 rad/s
    % tilt bandwidth -- cheap against zeta ~0.7 -- while attenuating
    % the broadband gyro floor into the torque command.  Filtered rate
    % is shared by tilt damping and the yaw gate; it resets whenever the
    % controller is not balancing so a restart does not inherit stale
    % state.
    filteredBodyRate = bodyRate;
    if isfield(p.controller.torqueDrive, "rateFilterTimeConstant") && ...
            p.controller.torqueDrive.rateFilterTimeConstant > 0
        persistent bodyRateFilter
        if isempty(bodyRateFilter) || mode ~= 1
            bodyRateFilter = bodyRate;
        else
            wRate = p.controller.sampleTime/ ...
                (p.controller.torqueDrive.rateFilterTimeConstant + ...
                 p.controller.sampleTime);
            bodyRateFilter = bodyRateFilter + ...
                wRate*(bodyRate - bodyRateFilter);
        end
        filteredBodyRate = bodyRateFilter;
    end
    state(5:7) = [filteredBodyRate(2); -filteredBodyRate(1); ...
        filteredBodyRate(3)];
    % Wheel-rate low-pass for the slip/propulsion channels: encoder
    % quantization flicker is ~3.7 rad/s per sample, while cruise rolling
    % rate is only ~5-6 rad/s, so the raw rate is >50% noise.  The raw
    % measurement thrashes the servo/damper torque to saturation (~50%
    % of samples at the clamp) and pumps ball z-spin through the
    % common-mode residual.  A short lag keeps the sustained component
    % these channels actually regulate.
    filteredWheelRate = wheelRate(:);
    if isfield(p.controller.torqueDrive, "wheelRateObserverEnabled") && ...
            p.controller.torqueDrive.wheelRateObserverEnabled && ...
            isfield(p.controller.torqueDrive, "wheelRateObserverGain") && ...
            p.controller.torqueDrive.wheelRateObserverGain > 0
        % Model-aided wheel-rate observer: prediction advances the
        % estimate with the measured wheel-drive dynamics (previous
        % torque command through driveGain, minus the compliance drag
        % toward the kinematic rolling rate), and the correction only
        % trims toward the quantized measurement.  Encoder flicker then
        % enters at observerGain (~0.15x) instead of at the lag filter's
        % pass-through (~0.17x but with no model prediction), so the
        % slip/propulsion/servo channels see the sustained rate without
        % either the flicker or the lag.
        persistent wheelRateObs prevObserverTorque prevObserverKinRate
        if isempty(wheelRateObs) || mode ~= 1
            wheelRateObs = wheelRate(:);
            prevObserverTorque = zeros(3, 1);
            prevObserverKinRate = wheelRate(:);
        else
            wheelRateObs = wheelRateObs + ...
                (p.controller.torqueDrive.wheelRateObserverDriveGain* ...
                 prevObserverTorque - ...
                 p.controller.torqueDrive.wheelRateObserverCompliance* ...
                 (wheelRateObs - prevObserverKinRate))* ...
                p.controller.sampleTime;
            wheelRateObs = wheelRateObs + ...
                p.controller.torqueDrive.wheelRateObserverGain* ...
                (wheelRate(:) - wheelRateObs);
        end
        filteredWheelRate = wheelRateObs;
    elseif isfield(p.controller.torqueDrive, "wheelRateFilterTimeConstant") && ...
            p.controller.torqueDrive.wheelRateFilterTimeConstant > 0
        persistent wheelRateFilter
        if isempty(wheelRateFilter) || mode ~= 1
            wheelRateFilter = wheelRate(:);
        else
            wWheel = p.controller.sampleTime/ ...
                (p.controller.torqueDrive.wheelRateFilterTimeConstant + ...
                 p.controller.sampleTime);
            wheelRateFilter = wheelRateFilter + ...
                wWheel*(wheelRate(:) - wheelRateFilter);
        end
        filteredWheelRate = wheelRateFilter;
    end
    % Yaw channel gating: the yaw-rate state error is fed to the LQI
    % when a yaw rate is commanded at rest (stop-window yaw in the
    % cycle profile, settle required as before) OR during commanded
    % translation (moveYaw -- the coupled (v, yaw) limiter upstream
    % bounds the shared wheel-rate budget, so tracking yaw while
    % moving no longer starves the tilt loop).  With no yaw command
    % the state is zeroed and the passive yaw damper below takes over;
    % while yaw IS commanded that damper must stay off or it brakes
    % the commanded rotation.
    yawGate = p.controller.torqueDrive.yawGate;
    stationary = norm(velocityBody) < yawGate.maxSpeed && ...
        tiltMagnitude < yawGate.maxTilt && commandSpeed == 0;
    yawActive = abs(command(3)) > yawGate.commandThreshold && ...
        (stationary || commandSpeed > 0);
    if yawActive
        state(7) = state(7) - yawRateCommand;
    else
        state(7) = 0;
    end
    % Velocity-state feedback: state(1:2) carries the BALL velocity
    % error in body frame, matching the state(7) measured-minus-
    % reference convention.  ballbotVelocityTorqueGains designs these
    % columns on a plant whose dv/dt = g*tilt + W'*tau/mEff, so -K*e
    % commands the contact force directly.  Gated to the post-recovery
    % settle window: in normal driving the force channel traction-drives
    % the coasting ball (stop-leg drift regression, ~0.2 m/s residual),
    % while during settle its damping is what keeps the drifting
    % assembly inside the tilt cone.  The reference is the pre-chase
    % user command: the chase rewrite exists for the wheel servo, not
    % for this loop.
    if settleWitness
        state(1:2) = ballVelocityBody - commandVelocityBodyUser;
    else
        state(1:2) = 0;
    end
    if tiltMagnitude > p.controller.tiltPriorityStart
        candidateIntegral = zeros(2, 1);
    end
    % A torque-mode velocity integral is only weakly observable at rest;
    % estimator startup transients otherwise wind it up into a permanent
    % torque bias that creeps the ball out from under the chassis.
    % Leak it toward zero so only sustained velocity errors persist.
    candidateIntegral = candidateIntegral* ...
        (1 - p.controller.torqueDrive.integralLeak* ...
         p.controller.sampleTime);
    if isfield(p.controller.torqueDrive, "integralEnabled") && ...
            ~p.controller.torqueDrive.integralEnabled
        candidateIntegral = zeros(2, 1);
    end
    if isfield(p.controller.torqueDrive, "wheelRateFeedbackEnabled") && ...
            ~p.controller.torqueDrive.wheelRateFeedbackEnabled
        state(8:10) = 0;
    end
    % Outer loop, channel 1: velocity error -> lean setpoint.  Wheel
    % torque is internal to the rover+ball system and cannot accelerate
    % the assembly centre of mass, so translation commands can only be
    % tracked through gravity: lean the chassis toward the desired
    % acceleration and let the tilt PD hold it there.
    velocityLeanGain = p.controller.torqueDrive.velocityLeanGain;
    if settleActive && isfield(p.controller.torqueDrive, ...
            "settleLeanGain") && ...
            p.controller.torqueDrive.settleLeanGain > 0
        % Settle-only brake gain: the IMU accel bias parks the measured
        % tilt ~0.016 rad above the real lean during drift, so the
        % demand must overshoot the bias before the real chassis leans
        % back against the drift.  Kept out of normal driving where a
        % hot lean gain would over-drive move legs.
        velocityLeanGain = p.controller.torqueDrive.settleLeanGain;
    end
    if velocityLeanGain ~= 0
        % Track the BALL velocity, not the rover velocity: the command
        % describes the assembly's translation and the ball is the
        % load-bearing element.  With rover velocity in the error, a
        % rover lead (relPos > 0) reads as "already at speed" while the
        % ball lags, so the lean demand eases exactly when the ball most
        % needs driving torque -- the observed runaway geometry.
        % ballVelocityBody is the estimate channel normally, and the
        % encoder-derived rate during elevated-authority settle (the
        % drift witness swap above -- the estimator's ball-velocity
        % channel runs ~7x hot on uncommanded drift).
        % The lean channel's error is the USER command, not the
        % follow-through chase rewrite -- during settle the rewritten
        % command equals ball speed and zeroes the only error signal
        % that can brake the drift.  Lean is the channel that draws a
        % real external (gravity) force on the assembly; the velocity
        % state gain adds fast contact-force response on the same
        % error (commandVelocityBodyUser).
        velocityErrorBody = commandVelocityBodyUser - ballVelocityBody;
        accelerationDemand = velocityLeanGain*velocityErrorBody;
        if settleActive && isfield(p.controller.torqueDrive, ...
                "settleLeanDamping")
            % D on the velocity outer loop: the P-only demand pumps a
            % growing drift oscillation (~7 s divergence at 10 deg
            % starts); opposing the measured ball acceleration rate
            % starves the overshoot that drives it.
            accelerationDemand = accelerationDemand - ...
                p.controller.torqueDrive.settleLeanDamping* ...
                settleKinAccel;
        end
        % Command-acceleration feedforward: a saturated P demand parks
        % the lean at its cap, which drives the ball ~3-5x harder than
        % the ramped command needs -- the ball overshoots the command,
        % the demand flips sign, and the momentum-transfer kicks ring the
        % tilt loop until it exits the cone (observed as accel-phase
        % divergence ~0.2-0.3 m/s at every traction setting).  Feeding
        % the command's own acceleration forward supplies the lean the
        % ramp actually needs, so the error term stays in its linear
        % range and only trims residual lag.  The USER command feeds
        % this, never the chase rewrite -- differentiating the chased
        % ball velocity during settle feeds the drift's own acceleration
        % back as a lean demand in the drift direction (the measured
        % ~-0.01 rad persistent lean that gravity-pumps the drift).
        if isfield(p.controller.torqueDrive, "accelFeedforwardGain") && ...
                p.controller.torqueDrive.accelFeedforwardGain ~= 0
            persistent prevCommandVelocity
            if isempty(prevCommandVelocity) || mode ~= 1
                prevCommandVelocity = commandVelocityBodyUser;
            end
            commandAccelBody = (commandVelocityBodyUser - ...
                prevCommandVelocity)/p.controller.sampleTime;
            prevCommandVelocity = commandVelocityBodyUser;
            accelerationDemand = accelerationDemand + ...
                p.controller.torqueDrive.accelFeedforwardGain* ...
                commandAccelBody;
        end
        leanLimit = p.controller.torqueDrive.leanSetpointLimit;
        accelerationDemand = min(max(accelerationDemand, -leanLimit* ...
            p.gravity), leanLimit*p.gravity);
        % Slew-rate limit on the lean setpoint: the ball can only roll
        % out from under the chassis at the contact-friction
        % acceleration, so a step lean demand tips the chassis past its
        % support before the ball can follow (measured: tilt 0.004 ->
        % 0.068 rad in 0.1 s at move onset, wheels unloading).  Rate-
        % limiting the demand lets the ball track the lean instead of
        % converting a step into a relPos->tilt runaway.
        if isfield(p.controller.torqueDrive, "leanRateLimit") && ...
                p.controller.torqueDrive.leanRateLimit > 0
            persistent accelerationDemandSlew
            if isempty(accelerationDemandSlew) || mode ~= 1
                accelerationDemandSlew = accelerationDemand;
            else
                stepLimit = p.controller.torqueDrive.leanRateLimit* ...
                    p.gravity*p.controller.sampleTime;
                demandDelta = accelerationDemand - ...
                    accelerationDemandSlew;
                demandDelta = min(max(demandDelta, -stepLimit), ...
                    stepLimit);
                accelerationDemandSlew = accelerationDemandSlew + ...
                    demandDelta;
            end
            accelerationDemand = accelerationDemandSlew;
        end
        % Cone guard: tilt beyond the commanded lean (overshoot,
        % recovery) gets no additional lean demand -- feeding a demand
        % the traction-limited torque cannot hold only adds tilt rate
        % bias on top of the runaway.
        if isfield(p.controller.torqueDrive, "leanFadeTilt")
            excessTilt = max(tiltMagnitude - leanLimit, 0);
            fade = max(1 - excessTilt/ ...
                p.controller.torqueDrive.leanFadeTilt, 0);
            accelerationDemand = fade*accelerationDemand;
        end
        % Eccentricity guard: the observed cycle fall is a tilt spike
        % the instant a move leg begins with the ball ~5 cm ahead of
        % the chassis (relPos < 0 along the command direction).
        % Accelerating onto an off-centre ball kicks a wheel off the
        % contact; throttling the lean demand while the geometry is
        % eccentric gives the contact time to recenter.
        if isfield(p.controller.torqueDrive, ...
                "accelRelPosGuardGain") && ...
                p.controller.torqueDrive.accelRelPosGuardGain ~= 0 && ...
                commandSpeed > 0.01
            commandDir = commandVelocityBody/commandSpeed;
            eccentricity = max(-dot(estimate(12:13), commandDir) - ...
                p.controller.torqueDrive.accelRelPosMargin, 0);
            guardFloor = 0.1;
            if isfield(p.controller.torqueDrive, ...
                    "accelRelPosGuardFloor")
                guardFloor = p.controller.torqueDrive. ...
                    accelRelPosGuardFloor;
            end
            guardScale = max(guardFloor, 1 - ...
                p.controller.torqueDrive.accelRelPosGuardGain* ...
                eccentricity);
            accelerationDemand = guardScale*accelerationDemand;
        end
        state(3:4) = state(3:4) - ...
            [accelerationDemand(1); -accelerationDemand(2)]/p.gravity;
    else
        accelerationDemand = zeros(2, 1);
    end
    % Static-lean (CG offset) adaptation: with a lateral CG offset the
    % true equilibrium is a tilted attitude, and holding tilt=0 leaves
    % the assembly drifting toward the offset (rest torque is ~zero --
    % the wedge sits on contact friction).  The drift velocity is the
    % offset signature: low-pass it and integrate it slowly into a lean
    % setpoint bias that tilts the equilibrium AWAY from the drift.
    % Under-biased -> drift toward the offset -> bias grows, a negative
    % feedback that self-stops at the true equilibrium.  The gain must
    % stay slow versus the lean->drift loop (~1 s): the injected lean
    % itself accelerates the assembly, and a fast integrator oscillates
    % (measured: gain ~1.5+ at 0.3 s filter diverges on 15-20 mm
    % offsets).  A deadband keeps the estimator's velocity noise floor
    % from winding the bias; the gate uses the ORIGINAL command because
    % stopFollow rewrites it to track the rolling ball.
    persistent staticLeanBias driftFilter driftRate prevDriftMag ...
        seenStill
    if isempty(staticLeanBias)
        staticLeanBias = zeros(2, 1);
        driftFilter = zeros(2, 1);
        driftRate = 0;
        prevDriftMag = 0;
        seenStill = false;
    end
    staticLeanAdapt = isfield(p.controller.torqueDrive, ...
        "staticLeanAdaptGain") && ...
        p.controller.torqueDrive.staticLeanAdaptGain ~= 0;
    % Inject the bias only at zero command: a wrongly-learned bias
    % must not steer the rover during commanded motion.  The bias is
    % already a tilt-setpoint in state(3:4) convention [pitch; -roll],
    % NOT a planar vector like accelerationDemand -- negating channel
    % 2 mirrored the roll lean away from a y-offset CG (positive
    % feedback: 20 mm y-offset fell at 2.6 s vs 4.7 s unadapted).
    if staticLeanAdapt && mode == 1 && norm(command(1:2)) < 0.01
        state(3:4) = state(3:4) - ...
            [staticLeanBias(1); staticLeanBias(2)];
    end
    % Tilt-gain scheduling: the static disturbance sensitivity is
    % ~1/(B*K_tilt - mgh/I), so a low tilt gain that stays quiet on IMU
    % noise also lets disturbance torques (propulsion reaction, contact
    % friction) walk the tilt past the ~0.016 rad hold cone.  Inside the
    % commanded lean band the low gain keeps small-signal torque well
    % under the clamp; past it the gain stiffens smoothly toward the
    % authority needed to catch a cone-edge excursion.  Scaling the
    % state equals scaling the tilt columns of the gain matrix.
    if isfield(p.controller.torqueDrive, "tiltStiffenGain")
        stiffenStart = p.controller.torqueDrive.leanSetpointLimit;
        stiffenBand = p.controller.torqueDrive.tiltStiffenTilt;
        boost = 1 + (p.controller.torqueDrive.tiltStiffenGain - 1)* ...
            min(max(tiltMagnitude - stiffenStart, 0)/stiffenBand, 1);
        state(3:6) = boost*state(3:6);
    end
    rawWheelCommand = -lqi.stateGain*state - ...
        lqi.integralGain*candidateIntegral;
    % Bounded leaky tilt-error integrator: at the heavy end the
    % proportional gain sits only ~5% above the static-hold floor, so
    % net stiffness B*K-gg is a few percent of nominal and constant
    % disturbance torques (contact friction, asymmetric wheel load,
    % relPos gravity moment) walk the tilt into the cone ~2-4 s into
    % every run.  The integrator slowly takes over the steady holding
    % load so the equilibrium returns to the commanded lean; the leak
    % plus state clamp keep it from winding up through saturation.
    if isfield(p.controller.torqueDrive, "tiltIntegralGain") && ...
            p.controller.torqueDrive.tiltIntegralGain ~= 0
        persistent tiltIntegral
        if isempty(tiltIntegral) || mode ~= 1
            tiltIntegral = zeros(2, 1);
        else
            tiltIntegral = tiltIntegral + p.controller.sampleTime* ...
                state(3:4);
            tiltIntegral = tiltIntegral* ...
                (1 - p.controller.torqueDrive.tiltIntegralLeak* ...
                 p.controller.sampleTime);
            tiltIntegral = min(max(tiltIntegral, ...
                -p.controller.torqueDrive.tiltIntegralLimit), ...
                p.controller.torqueDrive.tiltIntegralLimit);
        end
        rawWheelCommand = rawWheelCommand - ...
            p.controller.torqueDrive.tiltIntegralGain* ...
            lqi.stateGain(:, 3:4)*tiltIntegral;
    end
    if isfield(p.controller.torqueDrive, "leanHoldTorque") && ...
            velocityLeanGain ~= 0
        % Gravity-hold feedforward on the (slew-limited) lean demand:
        % pure feedback would need ~9x tilt amplification to reach the
        % demanded lean, drifting past the static hold cone.  The
        % feedforward supplies the hold torque for the DEMANDED lean --
        % smooth and noise-free -- while the feedback gain only
        % multiplies the noisy measured deviation.
        rawWheelCommand = rawWheelCommand + startupRamp( ...
            p.controller.torqueDrive.leanHoldTorque* ...
            [accelerationDemand(1); -accelerationDemand(2)]/p.gravity, ...
            enable, mode, p);
    end
    if ~yawActive
        % Common-mode removal: the tilt/balance channels are zero-sum
        % by design, but their residual mean pumps the ball's vertical
        % spin -- the unregulated mode whose accumulation ends every
        % move leg (~0.2 m/s) by unloading a wheel.  Strip it while no
        % yaw command is active; the intentional common-mode channels
        % (ball-spin brake, translation yaw damper) are added below.
        rawWheelCommand = rawWheelCommand - mean(rawWheelCommand);
    end
    % Traction control: a wheel that spins faster than the kinematic
    % rolling rate is slipping, so its torque cannot reach the ball.
    % Damping the slip rate keeps the contact engaged.  The deadzone
    % keeps encoder-quantization flicker (quantization/sampleTime ~3.7
    % rad/s) out of the damping channel.
    kinematicRate = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
        [velocityBody; bodyRate(3)];
    % Ball vertical spin brake: common-mode wheel torque is what the
    % rover-yaw damper uses to shed angular momentum, and it all lands
    % on the ball's free spin mode (nothing regulates omegaBall_z).
    % Left alone the spin accumulates ~1 rad/s^2 until the wheels need
    % lateral slip to stay engaged and the contact lets go around
    % |oBz| ~3.5 rad/s.  Braking the ball spin directly through the
    % common-mode channel stops the momentum dumping; the reaction on
    % the chassis also helps the yaw damper instead of fighting it.
    ballYawGain = p.controller.torqueDrive.ballYawGain;
    if ballYawGain ~= 0
        % +common torque accelerates the ball toward -z (measured), so
        % braking a -z spin needs torque of the same sign as oBz.  The
        % term is capped: the brake is common-mode on all three wheels,
        % so an uncapped request cancels the zero-sum tilt correction on
        % the wheels where they disagree and starves the attitude loop
        % exactly when a transition needs it most.
        ballYawTorque = ballYawGain*estimate(11);
        if isfield(p.controller.torqueDrive, "ballYawTorqueLimit")
            ballYawTorque = min(max(ballYawTorque, ...
                -p.controller.torqueDrive.ballYawTorqueLimit), ...
                p.controller.torqueDrive.ballYawTorqueLimit);
        end
        rawWheelCommand = rawWheelCommand + ballYawTorque*ones(3, 1);
    end
    % Passive yaw damper during translation: the gated yaw channel is
    % off while a translation command is active, and ball z-spin drags
    % the chassis yaw -- the drift rotates the commanded push direction
    % into a sideways escape.  A common-mode rate damper holds heading
    % while the ball-spin brake above absorbs the spin it pumps.  Gated
    % to active translation commands: at rest the same gain excites a
    % yaw/gyro-noise limit cycle.
    if isfield(p.controller.torqueDrive, "yawDampingGain") && ...
            p.controller.torqueDrive.yawDampingGain ~= 0 && ...
            ~yawActive && commandSpeed > 0
        rawWheelCommand = rawWheelCommand - ...
            p.controller.torqueDrive.yawDampingGain* ...
            filteredBodyRate(3)*ones(3, 1);
    end
    slipRate = filteredWheelRate - kinematicRate;
    if isfield(p.controller.torqueDrive, "filteredSlipDeadzone")
        slipDeadzone = p.controller.torqueDrive.filteredSlipDeadzone;
    else
        slipDeadzone = p.controller.torqueDrive.slipDeadzone;
    end
    slipActive = sign(slipRate).*max(abs(slipRate) - slipDeadzone, 0);
    rawWheelCommand = rawWheelCommand - ...
        p.controller.torqueDrive.slipDampingGain*slipActive;
    % Propulsion: kinematic wheel-rate servo.  Torque mode removed the
    % wheel-speed PI entirely, but translation still needs the wheels to
    % roll at the omni kinematic rate W*v.  A modest P term tracks that
    % target rate; the lean channel supplies the chassis' share of the
    % acceleration through gravity.  The deadzone keeps encoder flicker
    % (~3.7 rad/s) from biasing the propulsion torque.
    propulsionGain = p.controller.torqueDrive.propulsionGain;
    if propulsionGain ~= 0
        wheelRateCommand = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
            [commandVelocityBody; 0];
        rateError = wheelRateCommand - filteredWheelRate;
        rateActive = sign(rateError).*max(abs(rateError) - ...
            slipDeadzone, 0);
        % Propulsion yields to attitude: the holdable lean is only
        % ~0.023 rad of tilt with the full traction budget, so the
        % propulsion share is whatever the tilt/attitude channels leave
        % unused, hard-capped at propulsionTorqueLimit.  Exception: when
        % the tilt channel saturates the whole budget the rover wedges
        % into a static lean (wheels friction-locked, ball force cancels,
        % position frozen) -- a stall the pure leftover policy can never
        % exit.  propulsionMinShare reserves a minimum propulsion budget
        % even at full tilt saturation so the drive pattern can break the
        % wedge; the total torque then exceeds the nominal clamp by at
        % most the propulsion share.
        torqueLimit = min(p.driver.continuousTorqueLimit, ...
            p.controller.tractionLimitedWheelTorque);
        propTorque = min(max(propulsionGain*rateActive, ...
            -p.controller.torqueDrive.propulsionTorqueLimit), ...
            p.controller.torqueDrive.propulsionTorqueLimit);
        headroom = max(torqueLimit - abs(rawWheelCommand), 0);
        if isfield(p.controller.torqueDrive, "propulsionMinShare")
            headroom = max(headroom, ...
                p.controller.torqueDrive.propulsionMinShare*torqueLimit);
        end
        propTorque = sign(propTorque).*min(abs(propTorque), headroom);
        % Soft-start ramps only the DISTURBANCE paths (propulsion +
        % lean feedforward), never the tilt feedback: a settle-time
        % excursion needs full authority immediately -- ramping the
        % balance channel left a 0.023 rad transient uncatchable at 37%
        % authority (cycle_result_e05), while ramping propulsion removes
        % the wheelie kick that creates the excursion in the first
        % place.
        % Settle cuts the wheel servo entirely: commanding the rolling
        % rate makes the servo inject exactly the axle/bearing losses
        % as ball thrust -- a traction-drive pump that holds the drift
        % speed and keeps accelerating the ball (~+0.06 m/s^2 measured
        % during the va000 sweep).  Free-rolling wheels plus the
        % velocity-state force gain brake the drift instead.
        if settleWitness
            propTorque(:) = 0;
        end
        propTorque = startupRamp(propTorque, enable, mode, p);
        rawWheelCommand = rawWheelCommand + propTorque;
    end
    if elevatedAuthority && ...
            isfield(p.controller, "recoveryWheelTorqueLimit")
        torqueCap = min(p.driver.continuousTorqueLimit, ...
            p.controller.recoveryWheelTorqueLimit);
    else
        torqueCap = min(p.driver.continuousTorqueLimit, ...
            p.controller.tractionLimitedWheelTorque);
    end
    limit = torqueCap*ones(3, 1);
    wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    if mode == 2
        % Attitude-only recovery: the velocity (1:2), yaw-rate (7) and
        % wheel-rate (8:10) channels of the LQI state pump translation
        % while held (measured: a righted 10 deg start kept rolling at
        % ~0.2 m/s under recovery until the rover ran off the ball).
        % The wheel-rate columns (8:10) have no gains anyway, and the
        % velocity columns (1:2) -- which do carry the direct-force
        % velocity gain -- must NOT produce drive torque while the
        % attitude-only recovery law rights the chassis.
        stateRecovery = state;
        stateRecovery([1 2 7 8 9 10]) = 0;
        rawWheelCommand = -p.controller.recoveryGainScale* ...
            lqi.stateGain*stateRecovery - ...
            p.controller.torqueDrive.slipDampingGain*slipActive;
        wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    elseif mode == 0
        wheelSpeedCommand(:) = 0;
    end

    if staticLeanAdapt
        driftRaw = rotationBodyFromWorld*estimate(7:8);
        adaptGate = 0.30;
        if isfield(p.controller.torqueDrive, ...
                "staticLeanAdaptGate")
            adaptGate = p.controller.torqueDrive. ...
                staticLeanAdaptGate;
        end
        adaptRest = enable && mode == 1 && ...
            norm(command(1:2)) < 0.01 && norm(driftRaw) < adaptGate;
        % Qualification: learning is only armed by a rest period that
        % began from genuine stillness (ball below ~0.4 mm/s).  In the
        % move/stop cycle every stop window inherits a rolling ball --
        % its accelerating creep is deceleration-corner dynamics, not
        % a CG offset -- so the latch stays low and no bias is learned.
        % Once armed it stays armed through the creep until the next
        % commanded move or a non-mode-1 excursion.
        % Arming is decided on the rest ENTRY edge: the rest period
        % must begin from stillness.  Both channels are checked: the
        % lagging chassis estimate reads ~0 only from a settled start
        % at 5 ms, but at 2 ms it settles fast enough to read <0.02
        % while the ball still rolls -- the wheel-derived ball speed
        % covers that hole.  A cycle stop window always enters with
        % the ball rolling, so it can never arm: its accelerating
        % creep is deceleration-corner dynamics, not an offset.
        persistent prevAdaptRest
        if isempty(prevAdaptRest)
            prevAdaptRest = false;
        end
        if ~adaptRest
            seenStill = false;
        elseif ~prevAdaptRest
            % Judge stillness on RAW wheel rate, not the estimated
            % ball rate: estimate(9:10) is smoothed and reads low for
            % ~50 ms into a rest, which falsely armed learning inside
            % cycle stop windows at 2 ms.
            planarFromWheels = pinv(p.controller.lqi. ...
                wheelSpeedFromPlanarVelocity);
            wheelBallSpeed = norm( ...
                planarFromWheels(1:2, :)*wheelRate(:));
            seenStill = norm(driftRaw) < 0.02 && ...
                wheelBallSpeed < 0.03;
        end
        prevAdaptRest = adaptRest;
        adaptTau = 0.30;
        if isfield(p.controller.torqueDrive, ...
                "staticLeanAdaptTimeConstant")
            adaptTau = p.controller.torqueDrive. ...
                staticLeanAdaptTimeConstant;
        end
        if adaptRest
            driftW = p.controller.sampleTime/ ...
                (adaptTau + p.controller.sampleTime);
            driftFilter = driftFilter + driftW*(driftRaw - driftFilter);
            % Offset drift ACCELERATES under unopposed gravity; a
            % post-move coast DECELERATES under friction.  Without this
            % gate the bias learns the coasting velocity after every
            % move leg (cycle fell at ~15 s).  Track the filtered
            % drift magnitude's rate of change and only learn while the
            % drift is growing.
            driftMag = norm(driftFilter);
            instRate = (driftMag - prevDriftMag)/ ...
                p.controller.sampleTime;
            prevDriftMag = driftMag;
            driftRate = driftRate + driftW*(instRate - driftRate);
            adaptDeadband = 0.02;
            if isfield(p.controller.torqueDrive, ...
                    "staticLeanAdaptDeadband")
                adaptDeadband = p.controller.torqueDrive. ...
                    staticLeanAdaptDeadband;
            end
            % Offset creep is quasi-static (~0.1 m/s^2); a corner
            % runaway accelerates several times faster, so cap the
            % learning window on the fast side too.
            adaptMaxRate = 1.0;
            if isfield(p.controller.torqueDrive, ...
                    "staticLeanAdaptMaxRate")
                adaptMaxRate = p.controller.torqueDrive. ...
                    staticLeanAdaptMaxRate;
            end
            if driftMag > adaptDeadband && driftRate > 0 && ...
                    driftRate < adaptMaxRate && seenStill
                driftVec = driftFilter*(driftMag - adaptDeadband)/ ...
                    driftMag;
                staticLeanBias = staticLeanBias - ...
                    p.controller.torqueDrive.staticLeanAdaptGain* ...
                    p.controller.sampleTime*driftVec;
            end
            biasLimit = 0.10;
            if isfield(p.controller.torqueDrive, ...
                    "staticLeanAdaptLimit")
                biasLimit = p.controller.torqueDrive. ...
                    staticLeanAdaptLimit;
            end
            staticLeanBias = min(max(staticLeanBias, -biasLimit), ...
                biasLimit);
        else
            driftFilter(:) = 0;
            driftRate = 0;
            prevDriftMag = 0;
            % A contaminated bias (learned from transient creep in a
            % cycle stop window) decays while the rover is commanded:
            % only an offset that re-asserts itself every rest period
            % keeps its bias.  At a true offset hold the rest gate
            % never opens, so the learned equilibrium is retained.
            adaptLeak = 0.3;
            if isfield(p.controller.torqueDrive, ...
                    "staticLeanAdaptLeak")
                adaptLeak = p.controller.torqueDrive. ...
                    staticLeanAdaptLeak;
            end
            staticLeanBias = staticLeanBias* ...
                (1 - adaptLeak*p.controller.sampleTime);
        end
    end
    if mode == 1 && isfield(p.controller.torqueDrive, ...
            "wheelRateObserverEnabled") && ...
            p.controller.torqueDrive.wheelRateObserverEnabled
        % Feed the finalized command back to the wheel-rate observer's
        % prediction input (see filteredWheelRate above).
        prevObserverTorque = wheelSpeedCommand;
        prevObserverKinRate = kinematicRate;
    end
    if mode == 1
        % Torque residual deflects planar acceleration; the equivalent
        % velocity correction leaks into the velocity integrators.
        massEffective = p.rover.mass + p.ball.mass + ...
            p.ball.inertia(1, 1)/p.ball.radius^2;
        accelerationResidual = ...
            p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2).'* ...
            (wheelSpeedCommand - rawWheelCommand)/massEffective;
        candidateIntegral = candidateIntegral + p.controller.sampleTime* ...
            p.controller.lqi.antiWindupGain*accelerationResidual;
        integralLimit = p.controller.velocityIntegralLimit;
        nextVelocityIntegral = min(max(candidateIntegral, ...
            -integralLimit), integralLimit);
    else
        nextVelocityIntegral = zeros(2, 1);
    end
    % The velocity integral is the only force channel that can pump a
    % sustained drift: it winds to its limit against the residual ball
    % roll and keeps demanding torque after the direction has gone
    % unstable.  During the elevated-authority settle window the
    % follow-through brake already supplies the deceleration demand,
    % so freeze the integrator there.
    if mode == 1 && elevatedAuthority
        nextVelocityIntegral = zeros(2, 1);
    end
    if debugCapture
        persistent captureLog
        if isempty(captureLog)
            captureLog = zeros(0, 48);
        end
        captureLog(end+1, :) = [captureInputs.', state.', ...
            rawWheelCommand.', wheelSpeedCommand.', ...
            candidateIntegral.', double(mode), ...
            staticLeanBias.'];
        assignin('base', 'dbgLqiInputs', captureLog);
    end
    output = [wheelSpeedCommand; nextVelocityIntegral; double(mode); ...
        double(yawBiasReady)];
    return
end

% Planar tracking is introduced through the integral states so a velocity
% step cannot bypass the physical acceleration/lean limits. Yaw keeps its
% kinematic feedforward because it does not require a balancing lean.
feedforward = p.controller.lqi.wheelSpeedFromPlanarVelocity* ...
    [zeros(2, 1); yawRateCommand];
if tiltMagnitude > p.controller.tiltPriorityStart
    candidateIntegral = zeros(2, 1);
end
rawWheelCommand = feedforward - p.controller.lqi.stateGain*state - ...
    p.controller.lqi.trackingGainScale* ...
    p.controller.lqi.integralGain*candidateIntegral;
limit = p.motor.speedCommandLimit;
wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
if mode == 2
    rawWheelCommand = -p.controller.recoveryGainScale* ...
        p.controller.lqi.stateGain*state;
    wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
elseif mode == 0
    wheelSpeedCommand(:) = 0;
end

if mode == 1
    saturationCorrection = pinv( ...
        p.controller.lqi.wheelSpeedFromPlanarVelocity)* ...
        (wheelSpeedCommand - rawWheelCommand);
    candidateIntegral = candidateIntegral + p.controller.sampleTime* ...
        p.controller.lqi.antiWindupGain*saturationCorrection(1:2);
    integralLimit = p.controller.velocityIntegralLimit;
    nextVelocityIntegral = min(max(candidateIntegral, -integralLimit), ...
        integralLimit);
else
    nextVelocityIntegral = zeros(2, 1);
end
output = [wheelSpeedCommand; nextVelocityIntegral; double(mode); ...
    double(yawBiasReady)];
end

function scaled = startupRamp(value, enable, mode, p)
%STARTRUPRAMP Ramp a torque contribution in over startupTorqueRampTime.
%   Full authority from the first enabled step lets a settle-time kick
%   saturate the wheels (cycle_result_g05).  Applied to disturbance
%   paths only -- the tilt feedback must keep full authority or a
%   transient becomes uncatchable mid-ramp (cycle_result_e05).
if ~isfield(p.controller.torqueDrive, "startupTorqueRampTime") || ...
        p.controller.torqueDrive.startupTorqueRampTime <= 0
    scaled = value;
    return
end
persistent rampClock
if isempty(rampClock) || ~enable || mode == 0
    rampClock = 0;
else
    rampClock = rampClock + p.controller.sampleTime;
end
scaled = value*min(1, rampClock/ ...
    p.controller.torqueDrive.startupTorqueRampTime);
end
