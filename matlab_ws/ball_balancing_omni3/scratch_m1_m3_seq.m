% Sequential M1/M2/M3 verification driver.
% 1) post-cleanup cycle for bit-identical check vs cycle_result_m1_base
% 2) cycle runs at raised profile speeds (M2)
% 3) moveYaw benchmark (M3)
function scratch_m1_m3_seq()
cd(fileparts(mfilename('fullpath')));

scratch_cycle(49.5, 'm1_post');
a = load('cycle_result_m1_base.mat');
b = load('cycle_result_m1_post.mat');
fprintf('BIT_IDENTICAL=%d\n', isequal(a.res, b.res));
if ~isequal(a.res, b.res)
    f = fieldnames(a.res);
    for i = 1:numel(f)
        va = a.res.(f{i}); vb = b.res.(f{i});
        if isnumeric(va) && ~isequal(size(va), size(vb))
            fprintf('  %s: size differs\n', f{i});
        elseif isnumeric(va) && ~isequal(va, vb)
            fprintf('  %s: maxAbsDiff=%.3e\n', f{i}, ...
                max(abs(va(:) - vb(:))));
        end
    end
end

scratch_cycle(49.5, 'm2_cyc_v0195', ...
    {'command.profileVelocityWorld', [0.195; 0]});
scratch_cycle(49.5, 'm2_cyc_v024', ...
    {'command.profileVelocityWorld', [0.24; 0]});

r = ballbotBenchmark('moveYaw', 'm3_moveYaw');
disp(r.spec);
disp(r.pass);
end
