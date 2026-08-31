function imu = ballbotIdealImu(rotationWorldFromBody, ...
    accelerationWorld, angularVelocityBody, p)
%BALLBOTIDEALIMU Return ideal specific force and angular rate.

gravityWorld = [0; 0; -p.gravity];
specificForceBody = rotationWorldFromBody'*( ...
    accelerationWorld(:) - gravityWorld);
imu = [specificForceBody; angularVelocityBody(:)];
end
