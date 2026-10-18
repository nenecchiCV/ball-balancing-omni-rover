function ballbotRev1PlantAugment(model, p)
%BALLBOTREV1PLANTAUGMENT Runtime plant augmentation for the rev1 model.
%   Adds the ball-ground rolling-resistance load (External Force and
%   Torque on the ball frame driven by joint velocities and the sensed
%   normal force) and, when p.motor.outputShaftFrictionEnabled is set,
%   per-wheel smooth-Coulomb shaft friction via Ideal Torque Sources
%   across the DriveMotor rotor/case nets plus the matching DC-motor
%   viscous rebalance.
%
%   The change is applied at runtime instead of editing the .slx so the
%   file stays in R2024b format (saving from a newer MATLAB breaks the
%   Simscape Electrical driver blocks for R2024b).  Idempotent: returns
%   immediately once the rolling load exists (the friction additions
%   check their own block marker).  Do not save the model from a MATLAB
%   newer than R2024b.

if nargin < 1
    model = 'ball_balancing_omni3_multibody_lqi_rev1';
end
if ~bdIsLoaded(model)
    return;
end
frictionEnabled = nargin >= 2 && ~isempty(p) && ...
    isfield(p, 'motor') && isfield(p.motor, 'outputShaftFrictionEnabled') && ...
    p.motor.outputShaftFrictionEnabled;
ballSys = [model '/MultibodyPlant/Ball'];
if ~isempty(find_system(ballSys, 'SearchDepth', 1, ...
        'Name', 'BallRollingLoad'))
    rollingLoadDone = true;
else
    rollingLoadDone = false;
end
if ~rollingLoadDone
joint = [ballSys '/BallFreeJoint'];
contact = [ballSys '/BallGroundContact'];

% Joint signal port map: RConn1 is the follower frame; the physical
% signal outputs then interleave px,vx,py,vy,pz,vz,Q,w -> RConn2..9.
% The position/quaternion ports are already wired; vx,vy,vz,w sit free
% at RConn3,5,7,9.
jph = get_param(joint, 'PortHandles');
vxPort = jph.RConn(3); vyPort = jph.RConn(5);
vzPort = jph.RConn(7); wPort  = jph.RConn(9);

% Sensed ball-ground normal force scales the resistance (zero airborne).
% Toggle the sensing off->on and diff the port list: physical-signal
% ports expose no name property, so the newly appeared RConn IS fnm.
set_param(contact, 'SenseNormalForceMagnitude', 'off');
portsBefore = get_param(contact, 'PortHandles');
set_param(contact, 'SenseNormalForceMagnitude', 'on');
cph = get_param(contact, 'PortHandles');
nPort = setdiff(cph.RConn, portsBefore.RConn);
assert(isscalar(nPort), ...
    'ballbotRev1PlantAugment: could not isolate the fnm port');

cv = cell(1, 5);
cvPorts = [vxPort, vyPort, vzPort, wPort, nPort];
cvUnits = {'m/s', 'm/s', 'm/s', 'rad/s', 'N'};
cvNames = {'BallVxToSL', 'BallVyToSL', 'BallVzToSL', ...
    'BallSpinToSL', 'BallNormalToSL'};
for i = 1:5
    cv{i} = add_block('nesl_utility/PS-Simulink Converter', ...
        [ballSys '/' cvNames{i}], 'Unit', cvUnits{i});
end
mux = add_block('simulink/Signal Routing/Mux', ...
    [ballSys '/BallRollResMux'], 'Inputs', '5');
fcn = add_block('simulink/User-Defined Functions/MATLAB Fcn', ...
    [ballSys '/BallRollResFcn'], ...
    'MATLABFcn', 'ballbotRollingResistance(u,ballbotParams)');
dmx = add_block('simulink/Signal Routing/Demux', ...
    [ballSys '/BallRollResDemux'], 'Outputs', '[3 3]');
f2p = add_block('nesl_utility/Simulink-PS Converter', ...
    [ballSys '/BallRollResForceToPS'], 'Unit', 'N');
t2p = add_block('nesl_utility/Simulink-PS Converter', ...
    [ballSys '/BallRollResTorqueToPS'], 'Unit', 'N*m');
eft = add_block( ...
    'sm_lib/Forces and Torques/External Force and Torque', ...
    [ballSys '/BallRollingLoad'], ...
    'EnableForce', 'on', 'EnableTorque', 'on', ...
    'ForceResolutionFrame', 'World', ...
    'TorqueResolutionFrame', 'World');

cvPh = @(b) get_param(b, 'PortHandles');
for i = 1:5
    add_line(ballSys, cvPorts(i), cvPh(cv{i}).LConn(1), ...
        'autorouting', 'on');
    add_line(ballSys, cvPh(cv{i}).Outport(1), ...
        get_param(mux, 'PortHandles').Inport(i), 'autorouting', 'on');
end
add_line(ballSys, get_param(mux, 'PortHandles').Outport(1), ...
    get_param(fcn, 'PortHandles').Inport(1), 'autorouting', 'on');
add_line(ballSys, get_param(fcn, 'PortHandles').Outport(1), ...
    get_param(dmx, 'PortHandles').Inport(1), 'autorouting', 'on');
dph = get_param(dmx, 'PortHandles');
add_line(ballSys, dph.Outport(1), cvPh(f2p).Inport(1), ...
    'autorouting', 'on');
add_line(ballSys, dph.Outport(2), cvPh(t2p).Inport(1), ...
    'autorouting', 'on');
eph = get_param(eft, 'PortHandles');
add_line(ballSys, cvPh(f2p).RConn(1), eph.LConn(1));
add_line(ballSys, cvPh(t2p).RConn(1), eph.LConn(2));
% Attach the load to the ball frame: branch the joint follower port.
add_line(ballSys, jph.RConn(1), eph.RConn(1));

% Tidy layout: park the augmentation below the existing row.
set_param(eft, 'Position', [520 220 560 280]);
set_param(f2p, 'Position', [430 200 450 220]);
set_param(t2p, 'Position', [430 250 450 270]);
set_param(dmx, 'Position', [370 205 380 265]);
set_param(fcn, 'Position', [300 210 340 260]);
set_param(mux, 'Position', [240 200 250 270]);
for i = 1:5
    set_param(cv{i}, 'Position', [140 190+22*i 170 208+22*i]);
end
end

% Gearmotor output-shaft stiction: an Ideal Torque Source across the
% DriveMotor's rotor/case nets (which land on the RMI rod/case ports)
% applies a smooth Coulomb law of the sensed shaft velocity, giving the
% geartrain the breakaway deadzone the pure-viscous DC-motor model
% lacks.  A passive Rotational Friction element in this net leaves the
% RMI torque underdetermined at init (Simscape reports a configuration
% singularity), so the torque is imposed as a source instead; tanh
% keeps the transition continuous for the solver.  The DC Motor viscous
% coefficient is reduced to viscousShare of its value so the combined
% loss still matches the no-load-current measurement.  Idempotent via
% WheelShaftFricSrc.  Port map (probed): joint RConn3 is the velocity
% sense output that also feeds RMI.w; the torque source exposes
% LConn1=C(case), RConn1=S(signal in), RConn2=R(shaft).
if frictionEnabled
for wheelIndex = 1:3
    assy = sprintf('%s/MultibodyPlant/WheelAssembly%d', model, ...
        wheelIndex);
    if isempty(find_system(assy, 'SearchDepth', 1, ...
            'Name', 'WheelShaftFricSrc'))
        legacy = find_system(assy, 'SearchDepth', 1, ...
            'Name', 'WheelShaftFriction');
        for k = 1:numel(legacy)
            pho = get_param(legacy{k}, 'PortHandles');
            for f = fieldnames(pho)'
                for j = 1:numel(pho.(f{1}))
                    lh = get_param(pho.(f{1})(j), 'Line');
                    if ishandle(lh)
                        delete_line(lh);
                    end
                end
            end
            delete_block(legacy{k});
        end
        jph = get_param([assy sprintf('/Wheel%dJoint', wheelIndex)], ...
            'PortHandles');
        rph = get_param([assy '/RMI'], 'PortHandles');
        cph = get_param([assy '/CaseRef'], 'PortHandles');
        vin = add_block('nesl_utility/PS-Simulink Converter', ...
            [assy '/WheelShaftFricIn'], 'Unit', 'rad/s');
        fcn = add_block( ...
            'simulink/User-Defined Functions/MATLAB Fcn', ...
            [assy '/WheelShaftFricFcn'], ...
            'MATLABFcn', 'ballbotShaftFrictionTorque(u,ballbotParams)');
        t2p = add_block('nesl_utility/Simulink-PS Converter', ...
            [assy '/WheelShaftFricToPS'], 'Unit', 'N*m');
        src = add_block( ...
            'fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source', ...
            [assy '/WheelShaftFricSrc']);
        vph = get_param(vin, 'PortHandles');
        fph = get_param(fcn, 'PortHandles');
        tph = get_param(t2p, 'PortHandles');
        sph = get_param(src, 'PortHandles');
        add_line(assy, jph.RConn(3), vph.LConn(1), ...
            'autorouting', 'on');
        add_line(assy, vph.Outport(1), fph.Inport(1), ...
            'autorouting', 'on');
        add_line(assy, fph.Outport(1), tph.Inport(1), ...
            'autorouting', 'on');
        add_line(assy, tph.RConn(1), sph.RConn(1), ...
            'autorouting', 'on');
        add_line(assy, rph.RConn(2), sph.RConn(2), ...
            'autorouting', 'on');
        add_line(assy, cph.LConn(1), sph.LConn(1), ...
            'autorouting', 'on');
        set_param(src, 'Position', [560 400 600 440]);
        set_param(t2p, 'Position', [470 400 500 420]);
        set_param(fcn, 'Position', [380 395 430 435]);
        set_param(vin, 'Position', [290 400 320 420]);
    end
    set_param([assy '/DriveMotor'], 'lam', ...
        'ballbotParams.motor.viscousFriction*ballbotParams.motor.viscousShare');
end
else
% Disabled mid-session: strip the torque-source chain and restore the
% full motor viscous coefficient.
for wheelIndex = 1:3
    assy = sprintf('%s/MultibodyPlant/WheelAssembly%d', model, ...
        wheelIndex);
    stale = find_system(assy, 'SearchDepth', 1, 'RegExp', 'on', ...
        'Name', 'WheelShaftFric.*');
    for k = 1:numel(stale)
        pho = get_param(stale{k}, 'PortHandles');
        for f = fieldnames(pho)'
            for j = 1:numel(pho.(f{1}))
                lh = get_param(pho.(f{1})(j), 'Line');
                if ishandle(lh)
                    delete_line(lh);
                end
            end
        end
        delete_block(stale{k});
    end
    set_param([assy '/DriveMotor'], 'lam', ...
        'ballbotParams.motor.viscousFriction');
end
end

% Payload lateral offset (CG-offset robustness test): slide the
% payload-rod mount in the chassis frame.  set_param is unconditional
% so toggling the offset back to zero restores the centered mount on
% the next parameter build.  The rod's vertical mount offset stays at
% centerOffset.
mount = find_system(model, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Name', 'PayloadRodMount');
if ~isempty(mount) && nargin >= 2 && ~isempty(p) && ...
        isfield(p, 'fullplant') && isfield(p.fullplant, 'payload') && ...
        isfield(p.fullplant.payload, 'lateralOffset')
    off = p.fullplant.payload.lateralOffset;
    z = p.fullplant.payload.centerOffset;
    set_param(mount{1}, 'TranslationCartesianOffset', ...
        sprintf('[%.6g %.6g %.6g]', off(1), off(2), z));
end

% Wheel-rate measurement model: keep the encoder MATLAB Fcn on the
% per-sample backward difference.  Unconditional set so a stale
% block-level swap is always repaired.
rateBlock = find_system(model, 'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', 'Name', 'WheelRateCalculation');
if ~isempty(rateBlock) && nargin >= 2 && ~isempty(p) && ...
        isfield(p, 'encoder')
    set_param(rateBlock{1}, 'MATLABFcn', sprintf( ...
        'ballbotWheelRateFromDisplacement(u(1:3),u(4:6),ballbotParams.estimator.sampleTime,ballbotParams.encoder.quantization)'));
end
end
