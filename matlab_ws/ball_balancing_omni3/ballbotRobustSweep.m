function results = ballbotRobustSweep(scenario, modelName)
%BALLBOTROBUSTSWEEP Robustness sweep on one acceptance scenario.
%   RESULTS = BALLBOTROBUSTSWEEP(SCENARIO, MODELNAME) runs
%   ballbotBenchmark(SCENARIO) over a fixed set of plant variations
%   and returns the per-variant result cell array.  The acceptance
%   plan applies this sweep to ONE case only -- 'reverse' exercises
%   both translation directions plus the reversal guard at modest
%   runtime; extending the grid over every scenario multiplies
%   runtime without covering new failure physics.
%
%   Variants cover the documented sensitive axes:
%     nominal    - unmodified plant
%     cgX/cgY    - 20 mm payload lateral CG offset (static-lean
%                  adaptation case; worst measured CG disturbance)
%     imuYaw     - +0.02 rad/s yaw gyro bias (bias-learning case)
%
%   Measured on rev2 (reversalGuardSpeed=0.08):
%     reverse_nominal     survives, tilt 1.05 deg, 6/8
%     reverse_cgX+20mm    TIPS ~6.5 s -- a CG offset along the drive
%                         axis is uncompensated under command (the
%                         static-lean adapter only injects bias at
%                         ~zero command), and the instant +-0.30 leg
%                         exceeds the tilt envelope
%     reverse_cgY+20mm    survives, tilt 1.28 deg, 5/8
%     reverse_imuYaw+02   survives, tilt 1.21 deg, 4/8
%   On 'reverseMax' (+-0.40): only the nominal variant survives --
%   every plant variation tips inside the max-cap envelope.
%
%   Example:
%     ballbotRobustSweep()                        % reverse sweep
%     ballbotRobustSweep('reverseMax')            % max-cap envelope
if nargin < 1, scenario = 'reverse'; end
if nargin < 2, modelName = 'ball_balancing_omni3_multibody_lqi_rev2'; end
variants = {
    'nominal', {}
    'cgX+20mm', {{'fullplant.payload.lateralOffset', [0.02; 0]}}
    'cgY+20mm', {{'fullplant.payload.lateralOffset', [0; 0.02]}}
    'imuYaw+02', {{'simulation.imuYawBias', 0.02}}
};
results = cell(size(variants, 1), 1);
for i = 1:size(variants, 1)
    results{i} = ballbotBenchmark(scenario, ...
        sprintf('%s_%s', scenario, variants{i, 1}), variants{i, 2}, ...
        [], modelName);
end
end
