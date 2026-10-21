function scratch_ksweep(k, tagPrefix)
% Contact-stiffness robustness sweep: applies the same normal
% stiffness to wheel-ball and ball-ground contacts, with damping
% rescaled to near-critical for the respective contact masses
% (wheel ~0.5 kg, floor ~2.1 kg assembly share).  Runs the
% acceptance set at this stiffness point:
%   hold 5 s, translation/stop/yaw cycle 50 s, v040 rf008, roll10.
% Usage: scratch_ksweep(3e4, 'k3')
% Results land in the standard *_result_*.mat files plus
% ksweep_summary_<tagPrefix>.txt.
if nargin < 2, tagPrefix = sprintf('k%g', k/1e4); end
cwb = 2*sqrt(0.5*k);
cbg = 2*sqrt(2.1*k);
kov = {'contact.wheelBall.normalStiffness', k; ...
       'contact.wheelBall.normalDamping', cwb; ...
       'contact.ballGround.normalStiffness', k; ...
       'contact.ballGround.normalDamping', cbg};

scr = {@() scratch_cycle(5,  sprintf('%s_hold',  tagPrefix), kov); ...
       @() scratch_cycle(50, sprintf('%s_cyc',   tagPrefix), kov); ...
       @() scratch_speed(sprintf('%s_v040', tagPrefix), 0.40, 0.008, 17.2); ...
       @() scratch_lean10b(sprintf('%s_roll10', tagPrefix))};
% roll10 needs its own overrides merged with the stiffness overrides;
% run it inline instead of through scratch_lean10b so both apply.
scr{4} = @() scratch_cycle(40, sprintf('%s_roll10', tagPrefix), ...
    [kov; {'controller.torqueDrive.leanSetpointLimit', 0.015; ...
           'command.accelerationLimit', 0.06; ...
           'command.decelerationLimit', 0.03; ...
           'rover.initialEulerWorld', deg2rad([10; 0; 0])}]);
% v040 also needs the stiffness overrides merged.
scr{3} = @() scratch_cycle(17.2, sprintf('%s_v040', tagPrefix), ...
    [kov; {'command.cyclesEnabled', false; ...
           'command.velocityWorld', [0.40; 0]; ...
           'command.accelerationLimit', 0.06; ...
           'contact.ballGround.rollingFriction', 0.008}]);

for i = 1:numel(scr)
    try
        scr{i}();
    catch ME
        fprintf('[%s] step %d failed: %s\n', tagPrefix, i, ME.message);
    end
end

% Summarize the four result files with pass/tip metrics.
names = {'hold', 'cyc', 'v040', 'roll10'};
expect = [5, 50, 17.2, 40];   % nominal window lengths
fid = fopen(sprintf('ksweep_summary_%s.txt', tagPrefix), 'w');
fprintf(fid, 'k=%g N/m (cwb=%.0f cbg=%.0f)\n', k, cwb, cbg);
fprintf(fid, '%-7s %6s %8s %6s\n', 'scenario', 'tEnd', 'maxTilt', 'tTip');
for i = 1:numel(names)
    f = sprintf('cycle_result_%s_%s.mat', names{i}, tagPrefix);
    if ~isfile(f)
        fprintf(fid, '%-7s  MISSING\n', names{i});
        continue
    end
    s = load(f); s = s.res;
    tm = max(abs(s.tilt), [], 2);
    tt = s.tTip; if isinf(tt), tt = -1; end
    flag = '';
    if s.t(end) >= expect(i)-0.5 && tm(end) < 0.1 && tt < 0
        flag = 'PASS';
    end
    fprintf(fid, '%-7s %6.1f %8.4f %6.1f %s\n', ...
        names{i}, s.t(end), max(tm), tt, flag);
end
fclose(fid);
type(sprintf('ksweep_summary_%s.txt', tagPrefix));
end
