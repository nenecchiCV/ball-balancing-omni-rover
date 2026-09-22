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
    rampTime = min(speed/accel, p.command.cycleMoveDuration/2);
    if speed <= 0 && abs(yawTarget) > 0
        rampTime = min(abs(yawTarget)/accel, ...
            p.command.cycleMoveDuration/2);
    end
    if rampTime <= 0 || (speed <= 0 && yawTarget == 0)
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
        if phase < rampTime
            fraction = phase/rampTime;
        elseif phase < moveEnd - rampTime
            fraction = 1;
        elseif phase < moveEnd
            fraction = (moveEnd - phase)/rampTime;
        else
            fraction = 0;
        end
        command(1:2) = direction*speed*fraction;
        command(3) = yawTarget*fraction;
    end
else
    speed = norm(command(1:2));
    rampTime = speed/accel;
    if rampTime > 0 && time < rampTime
        command(1:2) = command(1:2)*(time/rampTime);
    end
end
end
