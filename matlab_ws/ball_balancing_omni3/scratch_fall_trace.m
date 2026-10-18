%SCRATCH_FALL_TRACE Examine the seconds before a cycle fall via dbgLqiInputs.
L = evalin('base', 'dbgLqiInputs');
nIn = size(L, 2) - 20;
est = L(:, 1:13);
cmd = L(:, 14:15);
st = L(:, nIn+(1:10));
tauC = L(:, nIn+10+(4:6));
mode = L(:, end-2);
n = size(L, 1);
t = (0:n-1)'*0.005;
% relPos estimate is est(12:13)
% find first mode ~= 1 near the end
idx = find(mode ~= 1 & t > 25, 1);
fprintf('first non-mode1 after 25s: r%d t=%.2f mode=%d\n', idx, t(idx), mode(idx));
for k = max(1, idx-300):50:min(n, idx+50)
    fprintf('t=%5.2f estV=[%+.2f %+.2f] ballV=[%+.2f %+.2f] rel=[%+.3f %+.3f] tilt=[%+.3f %+.3f] cmd=[%+.2f] tau=[%+.3f %+.3f %+.3f] mode=%d\n', ...
        t(k), est(k,7), est(k,8), 0.05*est(k,10), -0.05*est(k,9), ...
        est(k,12), est(k,13), est(k,1), est(k,2), cmd(k,1), ...
        tauC(k,1), tauC(k,2), tauC(k,3), mode(k));
end
