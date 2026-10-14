function ft = ballbotRollingResistance(u, p)
%BALLBOTROLLINGRESISTANCE Ball-ground rolling resistance force and torque.
%   u = [vx; vy; vz; wx; wy; wz; N]: ball centre velocity and angular
%   velocity resolved in the world frame, and the sensed ball-ground
%   normal force.  Returns [F; T] (6x1), the external load applied at
%   the ball reference frame, expressed in world coordinates.
%
%   A real hollow ball rolling on the floor dissipates energy through
%   contact-patch hysteresis: a horizontal drag opposing translation
%   plus a small scrubbing torque on the vertical spin.  The point
%   contact in the plant resists only sliding, so without this term a
%   coasting ball kept its momentum forever -- the missing dissipation
%   behind the deceleration runaway.  Both components scale with the
%   sensed normal force, so the load vanishes when the ball is airborne.

bg = p.contact.ballGround;
N = u(7);
if ~isfinite(N) || N < 0
    N = 0;
end
vxy = u(1:2);
sv = norm(vxy);
if sv > 0
    fxy = -(bg.rollingFriction*N*tanh(sv/bg.rollingCriticalVelocity))* ...
        vxy/sv;
else
    fxy = [0; 0];
end
tz = -bg.spinScrubArm*N*tanh(u(6)/bg.spinScrubCriticalRate);
ft = [fxy(1); fxy(2); 0; 0; 0; tz];
end
