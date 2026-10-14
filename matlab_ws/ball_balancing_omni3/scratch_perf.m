% Performance sweep harness: times a short rev1 sim under solver/contact
% variants.  Writes one summary line per case; results accumulate in
% perf_results.mat.
function scratch_perf(stopTime, cases)
if nargin < 1, stopTime = 2; end
if nargin < 2, cases = []; end
mdl = 'ball_balancing_omni3_multibody_lqi_rev1';
open_system(mdl);
% Silence the per-event solver warning flood so it cannot distort timing.
set_param(mdl, 'MaxConsecutiveMinStep', '100000');

p = ballbotRev1Parameters;
assignin('base', 'ballbotParams', p);

if isempty(cases)
    cases = {
        % tag, solverName, kBallGround, cBallGround, kWheelBall, cWheelBall, transBG, transWB, maxStep, localSolver
        'base',      'VariableStepAuto', [], [], [], [], [], [], [], 'off';
        'ode15s',    'ode15s',           [], [], [], [], [], [], [], 'off';
        'ode23t',    'ode23t',           [], [], [], [], [], [], [], 'off';
        'soft2',     'VariableStepAuto', 1.0e4, 270, 1.0e4, 140, [], [], [], 'off';
        'soft4',     'VariableStepAuto', 5.0e3, 190, 5.0e3, 100, [], [], [], 'off';
        'soft4x',    'VariableStepAuto', 5.0e3, 190, 5.0e3, 100, 2.0e-3, 3.0e-3, [], 'off';
        'soft4ms',   'VariableStepAuto', 5.0e3, 190, 5.0e3, 100, [], [], '1e-3', 'off';
        'localbe',   'VariableStepAuto', [], [], [], [], [], [], [], 'on';
        };
end

rows = {};
for i = 1:size(cases, 1)
    c = cases(i, :);
    tag = c{1};
    p2 = p;
    if ~isempty(c{3}), p2.contact.ballGround.normalStiffness = c{3}; end
    if ~isempty(c{4}), p2.contact.ballGround.normalDamping  = c{4}; end
    if ~isempty(c{5}), p2.contact.wheelBall.normalStiffness = c{5}; end
    if ~isempty(c{6}), p2.contact.wheelBall.normalDamping   = c{6}; end
    if ~isempty(c{7}), p2.contact.ballGround.transitionWidth = c{7}; end
    if ~isempty(c{8}), p2.contact.wheelBall.relaxedTransitionWidth = c{8}; end
    if numel(c) >= 11 && ~isempty(c{11})
        for k = 1:size(c{11}, 1)
            p2 = setfieldNested(p2, c{11}{k,1}, c{11}{k,2}); %#ok<SFLD>
        end
    end
    % keep static-deflection preload consistent with the new stiffness
    supportedMass = p2.ball.mass + p2.rover.mass;
    p2.fullplant.linearSpringDeflection = supportedMass*p2.gravity/ ...
        p2.contact.ballGround.normalStiffness;
    p2.fullplant.groundStaticDeflection = p2.fullplant.linearSpringDeflection;
    p2.rover.initialPositionWorld(3) = p2.ball.radius + ...
        p2.rover.centerAboveBall - p2.fullplant.groundStaticDeflection;

    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter('StopTime', num2str(stopTime), ...
        'SolverName', c{2});
    if ~isempty(c{9})
        in = in.setModelParameter('MaxStep', c{9});
    end
    in = in.setVariable('ballbotParams', p2);
    if numel(c) >= 12 && ~isempty(c{12})
        in = in.setModelParameter('SimulationMode', c{12});
    end
    sc = [mdl '/MultibodyPlant/Environment/SolverConfig'];
    if strcmp(c{10}, 'mb')
        set_param(sc, 'UseLocalSolver', 'off', ...
            'MultibodyUseLocalSolver', 'on', ...
            'MultibodyLocalSolverChoice', 'ODE1', ...
            'MultibodyLocalSolverSampleTime', '0.0005');
    else
        set_param(sc, 'UseLocalSolver', c{10}, ...
            'MultibodyUseLocalSolver', 'off');
    end

    st = struct('tag', tag, 'wall', nan, 'steps', 0, 'tEnd', 0, ...
        'maxTilt', nan, 'Nmin', nan, 'Nmax', nan, 'ballX', nan, ...
        'ballY', nan, 'tInit', nan, 'tExec', nan, 'tTerm', nan, 'err', "");
    try
        tic; out = sim(in); st.wall = toc;
        ti = out.SimulationMetadata.TimingInfo;
        st.tInit = ti.InitializationElapsedWallTime;
        st.tExec = ti.ExecutionElapsedWallTime;
        st.tTerm = ti.TerminationElapsedWallTime;
        tt = out.get('roverPose').Time;
        st.steps = numel(tt);
        st.tEnd = tt(end);
        tilt = hypot(out.get('roverPose').Data(:,4), ...
            out.get('roverPose').Data(:,5));
        st.maxTilt = max(tilt);
        N = [out.get('wheel1NormalForce').Data, ...
            out.get('wheel2NormalForce').Data, ...
            out.get('wheel3NormalForce').Data];
        st.Nmin = min(N(:)); st.Nmax = max(N(:));
        bp = out.get('ballPose');
        st.ballX = bp.Data(end,1); st.ballY = bp.Data(end,2);
    catch ME
        st.wall = toc; st.err = string(ME.message);
    end
    rows{end+1,1} = st; %#ok<AGROW>
    fprintf('%-10s wall=%6.1f init=%5.1f exec=%5.1f steps=%6d tEnd=%5.2f maxTilt=%.4f N=[%.1f %.1f] ball=(%.3f,%.3f) err=%s\n', ...
        tag, st.wall, st.tInit, st.tExec, getf(st,'steps',0), getf(st,'tEnd',0), ...
        getf(st,'maxTilt',nan), getf(st,'Nmin',nan), getf(st,'Nmax',nan), ...
        getf(st,'ballX',nan), getf(st,'ballY',nan), getf(st,'err',""));
end
set_param([mdl '/MultibodyPlant/Environment/SolverConfig'], ...
    'UseLocalSolver', 'off', 'MultibodyUseLocalSolver', 'off');

res = vertcat(rows{:});
if isfile('perf_results.mat')
    old = load('perf_results.mat'); res = [old.res; res];
end
save('perf_results.mat', 'res');
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

function s = setfieldNested(s, dottedPath, value)
parts = strsplit(dottedPath, '.');
sub(1) = struct('type', '.', 'subs', parts{1});
for i = 2:numel(parts)
    sub(i) = struct('type', '.', 'subs', parts{i}); %#ok<AGROW>
end
s = subsasgn(s, sub, value);
end
