function command = ballbotRev1CommandProfile(time, p)
%BALLBOTREV1COMMANDPROFILE Repeated move/stop velocity command for rev1.
%   With cyclesEnabled, the profile repeats a trapezoidal window of
%   profileVelocityWorld (with profileYawRate applied simultaneously) for
%   cycleMoveDuration seconds followed by a zero command for
%   cycleStopDuration seconds, cycleCount times, starting at cycleStart.
%   The planar magnitude and yaw rate ramp together at
%   command.accelerationLimit so the lean-limited balance loop can follow
%   the acceleration demand.
%   Otherwise the constant velocityWorld command applies, ramped in over
%   the same acceleration limit.

command = [p.command.velocityWorld(:); p.command.yawRate];
accel = p.command.accelerationLimit;
if isfield(p.command, "cyclesEnabled") && p.command.cyclesEnabled
    target = p.command.profileVelocityWorld(:);
    yawTarget = 0;
    if isfield(p.command, "profileYawRate")
        yawTarget = p.command.profileYawRate;
    end
    speed = norm(target);
    % Braking is asymmetric with acceleration: deceleration transfers
    % chassis momentum into the ball and the braking torque margin is
    % thinner than the drive margin, so a separate (gentler) limit keeps
    % the stop transient inside the hold cone.
    decel = accel;
    if isfield(p.command, "decelerationLimit")
        decel = p.command.decelerationLimit;
    end
    rampUp = min(speed/accel, p.command.cycleMoveDuration/2);
    rampDown = min(speed/decel, p.command.cycleMoveDuration/2);
    if speed <= 0 && abs(yawTarget) > 0
        rampUp = min(abs(yawTarget)/accel, ...
            p.command.cycleMoveDuration/2);
        rampDown = rampUp;
    end
    if rampUp <= 0 || (speed <= 0 && yawTarget == 0)
        command(1:2) = [0; 0];
        command(3) = 0;
        return
    end
    direction = [0; 0];
    if speed > 0
        direction = target/speed;
    end
    command(1:2) = [0; 0];
    command(3) = 0;
    period = p.command.cycleMoveDuration + p.command.cycleStopDuration;
    tEnd = p.command.cycleStart + p.command.cycleCount*period;
    if time >= p.command.cycleStart && time < tEnd
        phase = mod(time - p.command.cycleStart, period);
        moveEnd = p.command.cycleMoveDuration;
        if phase < rampUp
            fraction = phase/rampUp;
        elseif phase < moveEnd - rampDown
            fraction = 1;
        elseif phase < moveEnd
            fraction = (moveEnd - phase)/rampDown;
        else
            fraction = 0;
        end
        command(1:2) = direction*speed*fraction;
        % Yaw is time-separated from translation (the common-mode torque
        % that yaws the chassis also spins the ball, so doing both at
        % once drains the tilt loop's torque budget).  The yaw trapezoid
        % sits inside the stop window after cycleYawDelay, leaving a
        % settle margin before the next move phase.
        yawStart = moveEnd + p.command.cycleYawDelay;
        yawEnd = yawStart + p.command.cycleYawDuration;
        yawAllowed = true;
        if isfield(p.command, "cycleYawEvery") && ...
                p.command.cycleYawEvery > 0
            cycleIndex = floor((time - p.command.cycleStart)/period);
            yawAllowed = mod(cycleIndex, p.command.cycleYawEvery) == 0;
        end
        if yawAllowed && phase >= yawStart && phase < yawEnd
            yawRamp = min(abs(yawTarget)/accel, ...
                p.command.cycleYawDuration/2);
            yawPhase = phase - yawStart;
            if yawPhase < yawRamp
                yawFraction = yawPhase/yawRamp;
            elseif yawPhase < p.command.cycleYawDuration - yawRamp
                yawFraction = 1;
            else
                yawFraction = (p.command.cycleYawDuration - ...
                    yawPhase)/yawRamp;
            end
            command(3) = yawTarget*yawFraction;
        end
    end
else
    speed = norm(command(1:2));
    rampTime = speed/accel;
    if rampTime > 0 && time < rampTime
        command(1:2) = command(1:2)*(time/rampTime);
    end
end
% Bounded yaw pulse: sustained common-mode yaw pumps the ball's free
% z-spin mode and the plant loses a wheel ~0.6-0.8 rad into the turn,
% so demonstrator scenarios cap the total rotation per pulse instead of
% holding a rate indefinitely.  0/absent keeps the constant request.
% Applies to the non-cycle path only — the cycle profile already bounds
% yaw via its stop-window trapezoid (cycleYawDelay/cycleYawDuration).
if isfield(p.command, "yawPulseDuration") && ...
        p.command.yawPulseDuration > 0 && command(3) ~= 0 && ...
        ~(isfield(p.command, "cyclesEnabled") && p.command.cyclesEnabled)
    dur = p.command.yawPulseDuration;
    yawRamp = min(abs(command(3))/accel, dur/2);
    if time < yawRamp
        yawFraction = time/yawRamp;
    elseif time < dur - yawRamp
        yawFraction = 1;
    elseif time < dur
        yawFraction = (dur - time)/yawRamp;
    else
        yawFraction = 0;
    end
    command(3) = command(3)*yawFraction;
end
end
