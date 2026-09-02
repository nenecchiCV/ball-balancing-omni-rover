function p = ballbotParameters
%BALLBOTPARAMETERS Nominal plant, estimator, and controller parameters.

p.gravity = 9.80665;

% Ball candidate: 150 mm junior rhythmic-gymnastics rubber ball.
p.ball.radius = 0.075;
p.ball.mass = 0.300;
p.ball.inertia = (2/3)*p.ball.mass*p.ball.radius^2*eye(3);

% Nexus Robot 14108 omniwheel.
p.wheel.radius = 0.024;
p.wheel.width = 0.0251;
p.wheel.mass = 0.039;
p.wheel.rollerCount = 8;
p.wheel.azimuth = deg2rad([0; 120; 240]);
p.wheel.contactLatitude = deg2rad(45);

% Nexus Robot 16007 continuous-rotation servo.
p.servo.dimensions = [0.0417, 0.0197, 0.0429];
p.servo.mass = 0.055;
p.servo.maxTorque = 13*9.80665/100;
p.servo.minSpeed = 53*2*pi/60;
p.servo.maxSpeed = 62*2*pi/60;
p.servo.nominalVoltage = 5.0;
p.servo.operatingCurrent = 0.100;
p.servo.timeConstant = 0.030;

% Redesigned triangular body. The body frame origin is the IMU location.
p.rover.mass = 0.462;
p.rover.bodyMass = p.rover.mass - 3*(p.servo.mass + p.wheel.mass);
p.rover.centerAboveBall = 0.125;
p.rover.chassisRadius = 0.080;
p.rover.chassisHeight = 0.030;
p.rover.initialPositionWorld = [0; 0; ...
    p.ball.radius + p.rover.centerAboveBall];
p.rover.initialEulerWorld = deg2rad([0; 0; 0]);
p.rover.noseAxisBody = [1; 0; 0];

% Penalty contacts. Friction values are calibration starting points.
p.contact.wheelBall.normalStiffness = 2.0e5;
p.contact.wheelBall.normalDamping = 250;
p.contact.wheelBall.transitionWidth = 5.0e-4;
p.contact.wheelBall.staticFriction = 0.90;
p.contact.wheelBall.dynamicFriction = 0.75;
p.contact.wheelBall.rollerFriction = 0.02;
p.contact.wheelBall.criticalVelocity = 5.0e-3;
p.contact.ballGround.normalStiffness = 3.0e5;
p.contact.ballGround.normalDamping = 180;
p.contact.ballGround.transitionWidth = 5.0e-4;
p.contact.ballGround.staticFriction = 0.90;
p.contact.ballGround.dynamicFriction = 0.80;
p.contact.ballGround.criticalVelocity = 5.0e-3;

geometry = ballbotWheelGeometry(p);
p.wheel.normalLoadNominal = ...
    p.rover.mass*p.gravity/(3*sin(p.wheel.contactLatitude));
p.wheel.contactTorqueLimit = p.contact.wheelBall.dynamicFriction* ...
    p.wheel.normalLoadNominal*p.wheel.radius;
p.wheel.commandTorqueLimit = min(p.servo.maxTorque, ...
    p.wheel.contactTorqueLimit);
p.wheel.geometry = geometry;

% IMU/wheel estimator. State is [q_WB(4); v_WB(3); omegaBall_W(3);
% roverMinusBall_B_xy(2); gyroBiasZ; qualificationTime].
p.estimator.sampleTime = 0.005;
p.estimator.attitudeCorrectionGain = 2.5;
p.estimator.accelNormGate = 0.25*p.gravity;
p.estimator.velocityLeak = 0.9995;
p.estimator.ballRateTimeConstant = 0.030;
p.estimator.relativePositionLeak = 0.9998;
p.estimator.kinematicRegularization = 1.0e-8;
p.estimator.contactResidualScale = 0.25;

% Provisional yaw-gyro bias observer settings. These values must be tuned
% against the selected IMU noise, in-run bias stability, vibration, and
% encoder quantization before deployment on hardware.
p.estimator.biasWheelRateThreshold = 0.10;
p.estimator.biasAccelNormThreshold = 0.03*p.gravity;
p.estimator.biasRollPitchRateThreshold = 0.02;
p.estimator.biasYawRateThreshold = 0.05;
p.estimator.biasContactConfidenceThreshold = 0.80;
p.estimator.biasContactConfidenceRelease = 0.70;
p.estimator.biasQualificationTime = 0.50;
p.estimator.biasQualificationHysteresis = 1.25;
p.estimator.biasTimeConstant = 1.0;
p.estimator.biasMaximum = 0.10;
p.estimator.biasMaximumUpdatePerSample = 1.0e-4;

% Cascaded velocity/balance/yaw controller.
p.controller.sampleTime = p.estimator.sampleTime;
p.controller.velocityKp = [0.35; 0.35];
p.controller.velocityKi = [0.04; 0.04];
p.controller.relativePositionKp = [0; 0];
p.controller.velocityIntegralLimit = [0.20; 0.20];
p.controller.velocityIntegralLeak = 0.995;
p.controller.velocityConvergenceBand = 0.005;
p.controller.tiltPriorityStart = deg2rad(3);
p.controller.maxLean = deg2rad(4);
p.controller.balanceKp = [0.95; 0.95];
p.controller.balanceKd = [0.12; 0.12];
p.controller.yawRateKp = 0.08;
% Keep yaw actuation inhibited during startup calibration until the
% corrected yaw rate is within the provisional acceptance tolerance.
p.controller.yawBiasReadyRateThreshold = 0.002;
% A deliberate yaw command bypasses startup inhibition; low-motion bias
% learning remains disabled naturally once the wheels move.
p.controller.yawBiasCommandBypassThreshold = 1.0e-6;
p.controller.maxPlanarAcceleration = 0.60;
p.controller.maxSpeed = 0.12;
p.controller.maxYawRate = 0.80;
p.controller.recoveryTilt = deg2rad(18);
p.controller.fallenTilt = deg2rad(35);
p.controller.minimumContactConfidence = 0.20;
p.controller.recoveryGainScale = 1.35;

p.command.velocityWorld = [0.05; 0];
p.command.yawRate = 0;
p.command.enable = true;

p.simulation.stopTime = 4.0;
p.simulation.maxStep = 1.0e-4;
p.simulation.relativeTolerance = 1.0e-4;
p.simulation.imuYawBias = 0;
end
