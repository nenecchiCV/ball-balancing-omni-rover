function wheelRate = ballbotTimedEncoderRate( ...
    wheelDisplacement, previousWheelDisplacement, sampleTime, ...
    encoderQuantum)
%BALLBOTTIMEDENCODERRATE Input-capture (T-method hybrid) wheel rate.
%   Emulates a timer-input-capture quadrature read: count boundaries
%   crossed between previousWheelDisplacement and wheelDisplacement get
%   interpolated crossing times (sub-sample resolution, as a real
%   capture gives exact edge timestamps), and the rate is the hybrid
%   M/T value dc*quant/dtEdge-to-edge when boundaries cross, or the
%   decaying lower bound sign*quant/(t - t_lastEdge) while none do.
%   This removes the quant/Ts backward-difference flicker entirely:
%   a resting wheel sees its estimate decay to ~zero instead of
%   alternating 0 and +-quant/Ts, so the stillness gates and the
%   kinematic solve stop ingesting phantom rates.
%
%   Called per estimator sample; persistent edge state is reset by
%   clear functions (the benchmark/scratch scripts already do that
%   before each sim).

persistent lastCount lastEdgeTime dirSign holdRate callIndex
if isempty(callIndex)
    callIndex = 0;
    lastCount = round(wheelDisplacement(:)/encoderQuantum);
    lastEdgeTime = zeros(3, 1);
    dirSign = zeros(3, 1);
    holdRate = zeros(3, 1);
end
callIndex = callIndex + 1;
tNow = callIndex*sampleTime;

dispNow = wheelDisplacement(:);
dispPrev = previousWheelDisplacement(:);
countNow = round(dispNow/encoderQuantum);
for w = 1:3
    dc = countNow(w) - lastCount(w);
    if dc ~= 0
        % Interpolated time of the LAST crossed boundary.  For dc>0
        % the final boundary sits at countNow*quant; for dc<0 the
        % final boundary going down sits at (countNow+1)*quant.
        if dc > 0
            boundary = countNow(w)*encoderQuantum;
        else
            boundary = (countNow(w) + 1)*encoderQuantum;
        end
        travel = dispNow(w) - dispPrev(w);
        f = (boundary - dispPrev(w))/travel;
        f = min(max(f, 0), 1);
        tEdge = (callIndex - 1 + f)*sampleTime;
        dtEdge = tEdge - lastEdgeTime(w);
        if dtEdge > eps
            % dc counts over elapsed edge-to-edge time: at speed this
            % is the M/T hybrid rate; single-edge crossings give the
            % classic T-method period rate.
            holdRate(w) = dc*encoderQuantum/dtEdge;
        end
        lastEdgeTime(w) = tEdge;
        dirSign(w) = sign(dc);
    else
        % No new boundary: the true rate is BELOW the one-step bound
        % quant/dt since the last edge.  Report that decaying bound
        % (halved-sample floor so a stopped wheel reads ~0 quickly).
        if dirSign(w) ~= 0
            holdRate(w) = dirSign(w)*encoderQuantum/ ...
                max(tNow - lastEdgeTime(w), 0.5*sampleTime);
        end
    end
    lastCount(w) = countNow(w);
end
wheelRate = holdRate;
end
