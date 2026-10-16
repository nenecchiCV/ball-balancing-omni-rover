%SCRATCH_BIAS_TRACE Inspect learned staticLeanBias after a debugCapture run.
L = evalin('base', 'dbgLqiInputs');
bias = L(:, end-1:end);
est = L(:, 1:14);
cmd = L(:, 15:16);
db = vecnorm(diff(bias)/0.005, 2, 2);
upd = find(db > 1e-6);
fprintf('rows=%d updateRows=%d', size(L, 1), numel(upd));
if ~isempty(upd)
    fprintf(' first=%d last=%d\n', upd(1), upd(end));
    seg = [1; find(diff(upd) > 1) + 1];
    fprintf('update segments:');
    for s = 1:numel(seg)
        e = upd(end);
        if s < numel(seg)
            e = upd(seg(s+1) - 1);
        end
        fprintf(' [%d-%d]', upd(seg(s)), e);
    end
    fprintf('\n');
    for k = upd(1):40:upd(end)
        fprintf('r%-5d bias=[%+.3f %+.3f] estV=[%+.2f %+.2f] cmd=%+.2f\n', ...
            k, bias(k, 1), bias(k, 2), est(k, 7), est(k, 8), cmd(k, 1));
    end
else
    fprintf('\n');
end
