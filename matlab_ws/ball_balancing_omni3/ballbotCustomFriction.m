function frictionContact = ballbotCustomFriction( ...
    tangentVelocityContact, contactToWheelRotation, normalForce, p)
%BALLBOTCUSTOMFRICTION Anisotropic omniwheel friction in contact axes.
% The cylinder local z-axis is the wheel axle. The friction command is a
% 2-vector resolved in the Spatial Contact Force contact frame.

velocity = tangentVelocityContact(:);
rotation = reshape(contactToWheelRotation, 3, 3);
normalForce = max(normalForce, 0);

axleWheel = [0; 0; 1];
normalWheel = rotation(:, 3);
driveWheel = cross(axleWheel, normalWheel);
driveContact3 = rotation'*driveWheel;
driveContact = driveContact3(1:2);

driveNorm = norm(driveContact);
if driveNorm < 1.0e-9 || normalForce == 0
    frictionContact = zeros(2, 1);
    return
end

driveContact = driveContact/driveNorm;
rollerContact = [-driveContact(2); driveContact(1)];
driveSlip = driveContact'*velocity;
rollerSlip = rollerContact'*velocity;

criticalVelocity = p.contact.wheelBall.criticalVelocity;
driveLimit = p.contact.wheelBall.dynamicFriction*normalForce;
rollerLimit = p.contact.wheelBall.rollerFriction*normalForce;
driveForce = -driveLimit*tanh(driveSlip/criticalVelocity);
rollerForce = -rollerLimit*tanh(rollerSlip/criticalVelocity);
frictionContact = driveForce*driveContact + rollerForce*rollerContact;
end
