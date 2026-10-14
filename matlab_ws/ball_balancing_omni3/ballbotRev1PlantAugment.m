function ballbotRev1PlantAugment(model)
%BALLBOTREV1PLANTAUGMENT Runtime plant augmentation for the rev1 model.
%   Adds the ball-ground rolling-resistance load: an External Force and
%   Torque block on the ball frame, driven by the 6-DOF joint's sensed
%   velocities and the ball-ground contact normal force, computed by
%   ballbotRollingResistance.
%
%   The change is applied at runtime instead of editing the .slx so the
%   file stays in R2024b format (saving from a newer MATLAB breaks the
%   Simscape Electrical driver blocks for R2024b).  Idempotent: returns
%   immediately once BallRollingLoad exists.  Do not save the model from
%   a MATLAB newer than R2024b.

if nargin < 1
    model = 'ball_balancing_omni3_multibody_lqi_rev1';
end
if ~bdIsLoaded(model)
    return;
end
ballSys = [model '/MultibodyPlant/Ball'];
if ~isempty(find_system(ballSys, 'SearchDepth', 1, ...
        'Name', 'BallRollingLoad'))
    return;
end
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
