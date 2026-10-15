function imu = ballbotImuFromJoint(jointMeasurement, p)
%BALLBOTIMUFROMJOINT Convert 6-DOF Joint sensing into a 6-axis IMU.
% Input: [q_WB(4); accelerationWorld(3); angularVelocityBody(3)].
%
% Sensor realism (p.imu): when p.imu.enabled is set the ideal reading is
% corrupted with the mounted LSM6DSV16X's measured behaviour -- constant
% zero-rate bias, white noise at the datasheet density for the 200 Hz
% control-rate sampling, and word-length quantization.  A sinusoidal
% acceleration disturbance (p.imu.vibration*) injects frame vibration.
% Noise is drawn from a fixed-seed stream so a run is reproducible; the
% stream lives in `persistent` state, so `clear functions` (the scratch
% scripts already call it before every sim) restores the same sequence.

quaternion = jointMeasurement(1:4);
quaternion = quaternion/max(norm(quaternion), eps);
rotationWorldFromBody = quaternionToRotation(quaternion);
accelerationWorld = jointMeasurement(5:7);
angularVelocityBody = jointMeasurement(8:10);
angularVelocityBody(3) = angularVelocityBody(3) + ...
    p.simulation.imuYawBias;
imu = ballbotIdealImu(rotationWorldFromBody, accelerationWorld, ...
    angularVelocityBody, p);

if isfield(p, "imu") && isfield(p.imu, "enabled") && p.imu.enabled
    persistent noiseStream sampleIndex
    if isempty(noiseStream)
        noiseStream = RandStream("Threefry", "Seed", p.imu.noiseSeed);
        sampleIndex = 0;
    end
    sampleIndex = sampleIndex + 1;
    specificForceBody = imu(1:3) + p.imu.accelBias(:);
    gyroBody = imu(4:6) + p.imu.gyroBias(:);
    if p.imu.vibrationAmplitude > 0
        specificForceBody = specificForceBody + ...
            p.imu.vibrationAxis(:)*p.imu.vibrationAmplitude* ...
            sin(2*pi*p.imu.vibrationFrequencyHz* ...
            sampleIndex*p.estimator.sampleTime);
    end
    specificForceBody = specificForceBody + ...
        p.imu.accelNoiseDensity/sqrt(2*p.estimator.sampleTime)* ...
        randn(noiseStream, 3, 1);
    gyroBody = gyroBody + ...
        p.imu.gyroNoiseDensity/sqrt(2*p.estimator.sampleTime)* ...
        randn(noiseStream, 3, 1);
    if p.imu.accelQuantum > 0
        specificForceBody = p.imu.accelQuantum* ...
            round(specificForceBody/p.imu.accelQuantum);
    end
    if p.imu.gyroQuantum > 0
        gyroBody = p.imu.gyroQuantum*round(gyroBody/p.imu.gyroQuantum);
    end
    imu = [specificForceBody; gyroBody];
end
end

function rotation = quaternionToRotation(quaternion)
w = quaternion(1);
x = quaternion(2);
y = quaternion(3);
z = quaternion(4);
rotation = [1 - 2*(y^2 + z^2), 2*(x*y - z*w), 2*(x*z + y*w); ...
    2*(x*y + z*w), 1 - 2*(x^2 + z^2), 2*(y*z - x*w); ...
    2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x^2 + y^2)];
end
