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

if isfield(p.controller, "cascade") && ...
        isfield(p.controller.cascade, "enabled") && ...
        p.controller.cascade.enabled
    output = ballbotCascadeUpdate(estimate, command, ...
        previousVelocityIntegral, enable, previousYawBiasReady, ...
        biasDiagnostics, wheelRate, p);
    return
end

roll = estimate(1);
pitch = estimate(2);
yaw = estimate(3);
bodyRate = estimate(4:6);
velocityWorld = estimate(7:8);
contactConfidence = estimate(14);
tiltMagnitude = hypot(roll, pitch);

yawBiasReady = ballbotYawBiasStartupGuard(previousYawBiasReady ~= 0, ...
    bodyRate(3), biasDiagnostics(2) ~= 0, p);
torqueDrive = isfield(p.controller, "torqueDrive") && ...
    isfield(p.controller.torqueDrive, "enabled") && ...
    p.controller.torqueDrive.enabled;

if ~enable || tiltMagnitude >= p.controller.fallenTilt
    mode = uint8(0);
elseif tiltMagnitude >= p.controller.recoveryTilt || ...
        contactConfidence < p.controller.minimumContactConfidence
    mode = uint8(2);
else
    mode = uint8(1);
end

rotationBodyFromWorld = [cos(yaw), sin(yaw); ...
    -sin(yaw), cos(yaw)];
velocityBody = rotationBodyFromWorld*velocityWorld;
commandVelocityBody = rotationBodyFromWorld*command(1:2);
commandSpeed = norm(commandVelocityBody);
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
if torqueDrive && isfield(p.controller.torqueDrive, "stopFollowGain") ...
        && p.controller.torqueDrive.stopFollowGain > 0
    ballVelocityWorld0 = [p.ball.radius*estimate(10); ...
        -p.ball.radius*estimate(9)];
    ballSpeed0 = norm(ballVelocityWorld0);
    if commandSpeed < p.controller.torqueDrive.stopFollowGain*ballSpeed0
        commandVelocityBody = rotationBodyFromWorld* ...
            (p.controller.torqueDrive.stopFollowGain*ballVelocityWorld0);
        commandSpeed = norm(commandVelocityBody);
    end
end

if mode == 1
    % Integrate the BALL velocity error, matching the lean channel: the
    % command describes the assembly's translation and the ball is the
    % load-bearing element.  Rover velocity leads the ball during any
    % acceleration, so integrating rover error winds the integrator with
    % the transient lead every manoeuvre.
    velocityError = commandVelocityBody - rotationBodyFromWorld* ...
        [p.ball.radius*estimate(10); -p.ball.radius*estimate(9)];
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
    % requested at rest.  Damping residual yaw rate at standstill also
    % pumps the ball spin (measured: oBall_z ~21 rad/s inside 0.15 s
    % while parked at zero command), so the gate keys on the command
    % alone; uncommanded yaw drift is left to friction.
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
    yawGate = p.controller.torqueDrive.yawGate;
    stationary = norm(velocityBody) < yawGate.maxSpeed && ...
        tiltMagnitude < yawGate.maxTilt && commandSpeed == 0;
    yawActive = stationary && abs(command(3)) > yawGate.commandThreshold;
    if yawActive
        state(7) = state(7) - yawRateCommand;
    else
        state(7) = 0;
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
    if velocityLeanGain ~= 0
        % Track the BALL velocity, not the rover velocity: the command
        % describes the assembly's translation and the ball is the
        % load-bearing element.  With rover velocity in the error, a
        % rover lead (relPos > 0) reads as "already at speed" while the
        % ball lags, so the lean demand eases exactly when the ball most
        % needs driving torque -- the observed runaway geometry.
        ballVelocityWorld = [p.ball.radius*estimate(10); ...
            -p.ball.radius*estimate(9)];
        ballVelocityBody = rotationBodyFromWorld*ballVelocityWorld;
        velocityErrorBody = commandVelocityBody - ballVelocityBody;
        accelerationDemand = velocityLeanGain*velocityErrorBody;
        % Command-acceleration feedforward: a saturated P demand parks
        % the lean at its cap, which drives the ball ~3-5x harder than
        % the ramped command needs -- the ball overshoots the command,
        % the demand flips sign, and the momentum-transfer kicks ring the
        % tilt loop until it exits the cone (observed as accel-phase
        % divergence ~0.2-0.3 m/s at every traction setting).  Feeding
        % the command's own acceleration forward supplies the lean the
        % ramp actually needs, so the error term stays in its linear
        % range and only trims residual lag.
        if isfield(p.controller.torqueDrive, "accelFeedforwardGain") && ...
                p.controller.torqueDrive.accelFeedforwardGain ~= 0
            persistent prevCommandVelocity
            if isempty(prevCommandVelocity) || mode ~= 1
                prevCommandVelocity = commandVelocityBody;
            end
            commandAccelBody = (commandVelocityBody - ...
                prevCommandVelocity)/p.controller.sampleTime;
            prevCommandVelocity = commandVelocityBody;
            accelerationDemand = accelerationDemand + ...
                p.controller.torqueDrive.accelFeedforwardGain* ...
                commandAccelBody;
        end
        % Ball-tracking through the same lean channel: relPos = rover -
        % ball.  A positive offset (rover ahead of the ball) means the
        % lean has pushed the chassis beyond the ball, which is the
        % runaway geometry: the chassis must ease off acceleration so
        % the ball catches up.  The relative-velocity term damps the
        % loop -- position-only feedback on this coupled pair is a
        % delay-driven oscillator (same lesson as channel 2).
        if isfield(p.controller.torqueDrive, "positionLeanGain")
            relativeVelocityBody = rotationBodyFromWorld* ...
                (estimate(7:8) - ballVelocityWorld);
            accelerationDemand = accelerationDemand - ...
                p.controller.torqueDrive.positionLeanGain* ...
                estimate(12:13) - ...
                p.controller.torqueDrive.positionLeanDampingGain* ...
                relativeVelocityBody;
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
        state(3:4) = state(3:4) - ...
            [accelerationDemand(1); -accelerationDemand(2)]/p.gravity;
    else
        accelerationDemand = zeros(2, 1);
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
    % Outer loop, channel 2: station keeping.  relativePositionBody is
    % rover - ball; wheel torque redistributes it internally (ball +x,
    % chassis reaction -x), so a positive offset wants a positive-pattern
    % torque: d2s/dt2 = -beta*(torque coefficient), beta ~29 m/s^2 per
    % N*m.  The position term needs the relative-velocity term for
    % damping -- a pure position loop on this double-integrator is a
    % delay-driven oscillator, which is why both +/- signs of the old
    % undamped positionGain fell faster than no feedback.
    if isfield(p.controller.torqueDrive, "positionGain") && ...
            p.controller.torqueDrive.positionGain ~= 0
        ballVelocityWorld = [p.ball.radius*estimate(10); ...
            -p.ball.radius*estimate(9)];
        relativeVelocityBody = rotationBodyFromWorld* ...
            (estimate(7:8) - ballVelocityWorld);
        rawWheelCommand = rawWheelCommand + ...
            p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2)* ...
            (p.controller.torqueDrive.positionGain*estimate(12:13) + ...
             p.controller.torqueDrive.positionDampingGain* ...
             relativeVelocityBody);
    end
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
        % The ball must be torqued up through the wheel contacts; during
        % acceleration the chassis leads the ball and that offset maps
        % kinematically into pitch (tilt ~ relPos/0.12 m).  Closed on
        % the wheel-rate command instead of the lean channel so the
        % fast wheel->ball drive authority reels the ball forward:
        % rover ahead (relPos > 0) asks for MORE ball rolling rate.
        if isfield(p.controller.torqueDrive, "propulsionRelPosGain")
            ballVelocityWorld = [p.ball.radius*estimate(10); ...
                -p.ball.radius*estimate(9)];
            relativeVelocityBody = rotationBodyFromWorld* ...
                (estimate(7:8) - ballVelocityWorld);
            relCorrection = ...
                p.controller.torqueDrive.propulsionRelPosGain* ...
                estimate(12:13) + ...
                p.controller.torqueDrive.propulsionRelVelGain* ...
                relativeVelocityBody;
            wheelRateCommand = wheelRateCommand + ...
                p.controller.lqi.wheelSpeedFromPlanarVelocity(:, 1:2)* ...
                relCorrection;
        end
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
        propTorque = startupRamp(propTorque, enable, mode, p);
        rawWheelCommand = rawWheelCommand + propTorque;
    end
    limit = min(p.driver.continuousTorqueLimit, ...
        p.controller.tractionLimitedWheelTorque)*ones(3, 1);
    wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    % Torque slew limit: the LQI+bang-bang commands flip rail to rail
    % in one sample at cruise, and the resulting torque square wave is
    % what pumps the limit cycle past the ~0.13 m/s wall (raising the
    % traction budget makes it worse -- the amplitude, not the
    % resource, is the problem).  The real motor current cannot step
    % anyway (winding inductance), so rate-limiting the command is a
    % physically faithful damping of the relay.
    if isfield(p.controller.torqueDrive, "torqueSlewLimit") && ...
            p.controller.torqueDrive.torqueSlewLimit > 0 && mode == 1
        persistent prevTorqueCommand
        if isempty(prevTorqueCommand) || ~enable
            prevTorqueCommand = zeros(3, 1);
        end
        torqueStep = p.controller.torqueDrive.torqueSlewLimit* ...
            p.controller.sampleTime;
        wheelSpeedCommand = prevTorqueCommand + ...
            min(max(wheelSpeedCommand - prevTorqueCommand, ...
                -torqueStep), torqueStep);
        prevTorqueCommand = wheelSpeedCommand;
    end
    if mode == 2
        rawWheelCommand = -p.controller.recoveryGainScale* ...
            lqi.stateGain*state - ...
            p.controller.torqueDrive.slipDampingGain*slipActive;
        wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    elseif mode == 0
        wheelSpeedCommand(:) = 0;
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
    if debugCapture
        persistent captureLog
        if isempty(captureLog)
            captureLog = zeros(0, 46);
        end
        captureLog(end+1, :) = [captureInputs.', state.', ...
            rawWheelCommand.', wheelSpeedCommand.', ...
            candidateIntegral.', double(mode)];
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
