function ballbotRev1SimConfig(model)
%BALLBOTREV1SIMCONFIG Apply the fast-simulation configuration to the rev1 model.
%   BALLBOTREV1SIMCONFIG(model) applies the solver/H-bridge settings that the
%   rev1 Simscape plant needs for reasonable variable-step speed.  The model
%   file itself is kept in R2024b format (Simscape Electrical blocks cannot be
%   exported from newer MATLAB releases), so these settings are applied here
%   instead of being baked into the .slx.  Scripts that simulate the model
%   should call this after load_system; once the model is next saved from a
%   MATLAB that can still write R2024b, the same values may be stored in the
%   file directly.
%
%   Settings applied:
%     * MddDriver.regenerative_braking_mode = always_enabled (x3).  The real
%       Cytron MDD3A (TB67H450 H-bridge) freewheels through the FETs, i.e. it
%       is regenerative in both current directions.  The library default
%       'depends_on_rev' flips the averaged bridge between two equation modes
%       every time the armature current crosses zero -- which happens
%       constantly during balance correction -- and produced ~5700 Newton
%       exceptions per 2 s run.  always_enabled removes the mode switch:
%       exceptions fall to a few hundred and execution time by ~4x.
%     * MaxStep = 1e-3.  The solver's intrinsic step is ~4e-4 s (electrical
%       and contact time constants), so the old 2e-4 cap cost ~35% extra
%       steps with no accuracy benefit.

if nargin < 1 || isempty(model)
    model = 'ball_balancing_omni3_multibody_lqi_rev1';
end

load_system(model);
set_param(model, 'MaxStep', '1e-3');

for k = 1:3
    drv = sprintf('%s/MultibodyPlant/WheelAssembly%d/MddDriver', model, k);
    set_param(drv, 'regenerative_braking_mode', ...
        'ee.enum.converters.regenerative_braking_mode.always_enabled');
end
end
