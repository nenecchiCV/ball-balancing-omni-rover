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

if mode == 1
    velocityError = commandVelocityBody - velocityBody;
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
        velocityErrorBody = commandVelocityBody - velocityBody;
        accelerationDemand = velocityLeanGain*velocityErrorBody;
        % Ball-tracking through the same lean channel: relPos = rover -
        % ball.  A positive offset (rover ahead of the ball) means the
        % lean has pushed the chassis beyond the ball, which is the
        % runaway geometry: the chassis must ease off acceleration so
        % the ball catches up.  The relative-velocity term damps the
        % loop -- position-only feedback on this coupled pair is a
        % delay-driven oscillator (same lesson as channel 2).
        if isfield(p.controller.torqueDrive, "positionLeanGain")
            ballVelocityWorld = [p.ball.radius*estimate(10); ...
                -p.ball.radius*estimate(9)];
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
        state(3:4) = state(3:4) - ...
            [accelerationDemand(1); -accelerationDemand(2)]/p.gravity;
    end
    rawWheelCommand = -lqi.stateGain*state - ...
        lqi.integralGain*candidateIntegral;
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
        % braking a -z spin needs torque of the same sign as oBz.
        rawWheelCommand = rawWheelCommand + ballYawGain*estimate(11)* ...
            ones(3, 1);
    end
    slipRate = wheelRate(:) - kinematicRate;
    slipDeadzone = p.controller.torqueDrive.slipDeadzone;
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
        rateError = wheelRateCommand - wheelRate(:);
        rateActive = sign(rateError).*max(abs(rateError) - ...
            slipDeadzone, 0);
        % Propulsion yields to attitude: the holdable lean is only
        % ~0.023 rad of tilt with the full traction budget, so the
        % propulsion share is whatever the tilt/attitude channels leave
        % unused, hard-capped at propulsionTorqueLimit.
        torqueLimit = min(p.driver.continuousTorqueLimit, ...
            p.controller.tractionLimitedWheelTorque);
        propTorque = min(max(propulsionGain*rateActive, ...
            -p.controller.torqueDrive.propulsionTorqueLimit), ...
            p.controller.torqueDrive.propulsionTorqueLimit);
        headroom = max(torqueLimit - abs(rawWheelCommand), 0);
        propTorque = sign(propTorque).*min(abs(propTorque), headroom);
        rawWheelCommand = rawWheelCommand + propTorque;
    end
    limit = min(p.driver.continuousTorqueLimit, ...
        p.controller.tractionLimitedWheelTorque)*ones(3, 1);
    wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    if mode == 2
        rawWheelCommand = -p.controller.recoveryGainScale* ...
            lqi.stateGain*state - ...
            p.controller.torqueDrive.slipDampingGain*slipActive;
        wheelSpeedCommand = min(max(rawWheelCommand, -limit), limit);
    elseif mode == 0
        wheelSpeedCommand(:) = 0;
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
            candidateIntegral.', double(mode)]; %#ok<AGROW>
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
