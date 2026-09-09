function command = ballbotCommandProfile(time, p)
%BALLBOTCOMMANDPROFILE Finite velocity command used by full-plant tests.

command = [p.command.velocityWorld(:); p.command.yawRate];
% if isfield(p.command, "profileEnabled") && p.command.profileEnabled
%     command = zeros(3, 1);
%     if time >= p.command.profileStart && time < p.command.profileStop
%         command(1:2) = p.command.profileVelocityWorld(:);
%     end
% end
% end
