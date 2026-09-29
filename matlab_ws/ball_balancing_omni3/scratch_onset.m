% Divergence onset analysis: do wheel normal forces drop before tilt grows?
function scratch_onset(tag)
if nargin < 1, tag = 'f05'; end
s = load(['cycle_result_' tag '.mat']);
r = s.res;
t = r.t;
iD = find(r.tilt > 0.02, 1);
t0 = t(iD);
fprintf('tag=%s onset t=%.3f tilt=%.4f nT=%d nN=%d nDuty=%d nEst=%d\n', ...
    tag, t0, r.tilt(iD), numel(t), size(r.N, 1), size(r.duty, 1), ...
    size(r.est, 1));
for tt = (t0 - 0.15):0.02:(t0 + 0.15)
    kp = find(t >= tt, 1);
    kn = min(round(tt*size(r.N, 1)/t(end)) + 1, size(r.N, 1));
    kd = min(round(tt*size(r.duty, 1)/t(end)) + 1, size(r.duty, 1));
    ke = min(round(tt/0.005) + 1, size(r.est, 1));
    fprintf(['t=%.2f tilt=%.4f N=(%.2f,%.2f,%.2f) ', ...
        'duty=(%.2f,%.2f,%.2f) eTilt=%.4f\n'], tt, r.tilt(kp), ...
        r.N(kn, 1), r.N(kn, 2), r.N(kn, 3), ...
        r.duty(kd, 1), r.duty(kd, 2), r.duty(kd, 3), ...
        hypot(r.est(ke, 1), r.est(ke, 2)));
end
end
