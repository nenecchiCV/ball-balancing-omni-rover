function [compPos, compNeg] = ballbotDeadzoneAdaptStep( ...
    duty, filteredSpeed, compPosInit, compNegInit, enable, p)
%BALLBOTDEADZONEADAPTSTEP Online residual-deadzone adaptation (PoC).
%   NOT wired into ballbotWheelServoDutyStep: sim evaluation showed
%   the always-on estimator ratchets comp on ordinary reversals and
%   destabilises even nominal runs (36 deg tip in hold), so the call
%   was removed and no p.drive flag re-enables it.  Kept as the
%   experiment reference; re-wiring it requires a gated,
%   controlled-identification design, not always-on operation.
%
%   Runs at the 1 kHz inner task.  Event-driven estimator: every wheel
%   reversal is a passive TUNE_DEADZONE mini-sweep.  The wheel arms
%   while |speed| < adaptArmRate and, on the first crossing of
%   adaptWakeRate (matching the firmware sweep's 2 rad/s wake
%   threshold), samples the compensated output duty.  That duty is the
%   plant deadzone plus the small net torque actually moving the
%   wheel, so comp tracks it with an event EMA -- slightly biased
%   high, which is the SAFE direction: the robustness sweep showed
%   under-compensation >~+25% is fatal while ~2x over-compensation
%   only degrades tracking.
%
%   Detection rules that keep ordinary motion from corrupting the
%   estimate:
%     * the onset event only counts when the drive sign matches the
%       motion sign (ball-driven back-drive is not a deadzone event);
%     * comp moves at most adaptStep per event toward the measured
%       duty, so a single bad sample cannot destabilize the loop;
%     * comp is bounded to [0, p.drive.adaptMax] (param_store clamps
%       at 0.3);
%     * state re-initialises from the identified comp vectors when
%       enable is low (sim start / standby).
%
%   Earlier versions -- instantaneous stuck ramp, accel-gated stuck
%   ramp, and stuck-duty-cycle score -- all ratcheted comp upward on
%   normal reversals/startup transients and destabilized the nominal
%   case.  The passive-onset estimator only updates on unambiguous
%   breakaway events, which normal balance operation produces at a
%   low rate by definition.
%
%   Firmware port note: mirrors a wheel_servo-side adaptive update;
%   persistence would land in the param_store-backed
%   deadzone_comp fields or a RAM shadow applied on tune/save.

persistent pos neg armed
if isempty(pos) || ~enable
    pos = compPosInit(:);
    neg = compNegInit(:);
    armed = true(3, 1);
end
d = p.drive;
wArm = optField(d, 'adaptArmRate', 0.5);
step = optField(d, 'adaptStep', 0.03);
margin = optField(d, 'adaptMargin', 1.0);
cMax = optField(d, 'adaptMax', 0.3);
for i = 1:3
    speed = filteredSpeed(i);
    if abs(speed) < wArm
        armed(i) = true;
    elseif armed(i)
        % Breakaway: the wheel just left the stopped band, so the
        % compensated duty RIGHT NOW approximates the deadzone plus a
        % small net torque.  Sampling here -- not at a higher wake
        % threshold -- minimises the command-slew bias: during fast
        % torque ramps the duty can far exceed the deadzone by the
        % time the wheel reaches 2 rad/s, which inflated the estimate
        % and ratcheted comp into the over-compensated region.
        armed(i) = false;
        comp = pos(i);
        if duty(i) < 0
            comp = neg(i);
        end
        effDuty = duty(i) + sign(duty(i))*comp;
        if sign(effDuty) == sign(speed) && effDuty ~= 0
            measured = min(abs(effDuty)*margin, cMax);
            % One-sided update: comp only ever grows.  Small effDuty
            % samples are unreliable -- the rolling ball back-drives
            % the wheels through the contact patch, so a wheel can
            % leave the stopped band with almost no motor command --
            % and decreasing comp into the under-compensated region
            % is the FATAL direction (sweep: +25% residual falls,
            % ~2x over-comp merely degrades).  Overshoot is recovered
            % by periodic TUNE_DEADZONE re-identification instead.
            if duty(i) > 0 && measured > pos(i)
                pos(i) = min(pos(i) + step, measured);
            elseif duty(i) < 0 && measured > neg(i)
                neg(i) = min(neg(i) + step, measured);
            end
        end
    end
end
compPos = pos;
compNeg = neg;
end

function v = moveToward(current, target, maxStep)
v = current + min(max(target - current, -maxStep), maxStep);
end

function v = optField(s, name, fallback)
if isfield(s, name)
    v = s.(name);
else
    v = fallback;
end
end
