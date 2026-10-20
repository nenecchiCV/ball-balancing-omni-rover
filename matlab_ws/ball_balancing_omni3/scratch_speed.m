function scratch_speed(tag, v, rf, stopTime, accel)
% Constant-command speed feasibility run: cyclesEnabled off, the
% non-cycle path ramps command.velocityWorld at accelerationLimit and
% holds it.  rf overrides ball-ground rollingFriction (floor band
% 0.005-0.015).  accel optionally overrides command.accelerationLimit.
% stopTime defaults to ramp + 10 s cruise.
if nargin < 4, stopTime = 0; end
if nargin < 5, accel = 0.06; end
if stopTime <= 0
    stopTime = 0.5 + v/accel + 10;
end
ov = {'command.cyclesEnabled', false; ...
      'command.velocityWorld', [v; 0]; ...
      'command.accelerationLimit', accel; ...
      'contact.ballGround.rollingFriction', rf};
scratch_cycle(stopTime, tag, ov);
end
