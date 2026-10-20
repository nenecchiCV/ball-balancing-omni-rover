function scratch_dbg(tag, v, rf, stopTime)
% Debug-capture speed run: same as scratch_speed but enables the
% controller's dbgLqiInputs capture (estimate, command, state, raw and
% final wheel torque, candidate integral, mode, static-lean bias per
% sample).  Saves the capture next to the usual cycle_result mat.
if nargin < 4, stopTime = 0; end
if stopTime <= 0
    stopTime = 0.5 + v/0.06 + 10;
end
ov = {'command.cyclesEnabled', false; ...
      'command.velocityWorld', [v; 0]; ...
      'contact.ballGround.rollingFriction', rf; ...
      'controller.torqueDrive.debugCapture', true};
scratch_cycle(stopTime, tag, ov);
dbg = evalin('base', 'dbgLqiInputs');
save(['cycle_dbg_' tag '.mat'], 'dbg');
end
