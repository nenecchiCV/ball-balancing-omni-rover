function p = ballbotParameters
%BALLBOTPARAMETERS Nominal plant, estimator, and controller parameters.

p.gravity = 9.80665;

% Reference ball: 100 mm, 0.285 kg rigid ball with thin-shell inertia.
p.ball.radius = 0.050;
p.ball.mass = 0.285;
p.ball.inertia = (2/3)*p.ball.mass*p.ball.radius^2*eye(3);

% Nexus Robot 14108 omniwheel.
p.wheel.radius = 0.024;
p.wheel.width = 0.0251;
p.wheel.mass = 0.039;
p.wheel.rollerCount = 8;
p.wheel.azimuth = deg2rad([0; 120; 240]);
p.wheel.contactLatitude = deg2rad(55);
% Small geometric preload keeps all three nominally tangent contacts
% active despite contact-detection and floating-point tolerances.
p.wheel.contactPreload = 5.0e-5;

% DFRobot FIT0521 6 V geared DC motor with quadrature Hall encoder.
p.motor.dimensions = [0.0520, 0.0244, 0.0244];
p.motor.mass = 0.096;
p.motor.nominalVoltage = 6.0;
p.motor.gearRatio = 34.02;
p.motor.noLoadSpeed = 210*2*pi/60;
p.motor.noLoadCurrent = 0.13;
p.motor.stallTorque = 10*9.80665/100;
p.motor.stallCurrent = 3.2;
p.motor.armatureResistance = ...
    p.motor.nominalVoltage/p.motor.stallCurrent;
p.motor.torqueConstant = p.motor.stallTorque/p.motor.stallCurrent;
p.motor.backEmfConstant = ...
    (p.motor.nominalVoltage - ...
    p.motor.noLoadCurrent*p.motor.armatureResistance)/ ...
    p.motor.noLoadSpeed;
p.motor.viscousFriction = ...
    p.motor.torqueConstant*p.motor.noLoadCurrent/p.motor.noLoadSpeed;
p.motor.timeConstant = 0.030;
p.motor.equivalentInertia = p.motor.timeConstant* ...
    p.motor.stallTorque/p.motor.noLoadSpeed;

% Two Cytron MDD3A boards provide the three required PWM/DIR channels.
p.driver.boardCount = 2;
p.driver.channelCountPerBoard = 2;
p.driver.supplyVoltage = p.motor.nominalVoltage;
p.driver.continuousCurrent = 3.0;
p.driver.peakCurrent = 5.0;
p.driver.pwmFrequency = 20e3;
p.driver.maxDutyCycle = 0.95;
p.driver.continuousTorqueLimit = p.motor.torqueConstant* ...
    min(p.driver.continuousCurrent, p.motor.stallCurrent);
p.driver.peakTorqueLimit = p.motor.stallTorque;

p.encoder.supplyVoltage = 3.3;
p.encoder.pulsesPerOutputRevolution = 341.2;
p.encoder.quantization = 2*pi/p.encoder.pulsesPerOutputRevolution;

% Output-shaft-equivalent speed loop.  The 30 ms time constant is
% provisional until a FIT0521 step response is measured under load.
p.motor.speedControllerNaturalFrequency = 30;
p.motor.speedControllerDamping = 0.90;
p.motor.speedPlantDcGain = ...
    p.motor.noLoadSpeed/p.driver.supplyVoltage;
p.motor.speedControllerKp = ...
    (2*p.motor.speedControllerDamping* ...
    p.motor.speedControllerNaturalFrequency*p.motor.timeConstant - 1)/ ...
    p.motor.speedPlantDcGain;
p.motor.speedControllerKi = ...
    p.motor.timeConstant*p.motor.speedControllerNaturalFrequency^2/ ...
    p.motor.speedPlantDcGain;
p.motor.speedControllerAntiWindup = 1/max(p.motor.speedControllerKp, eps);
p.motor.speedControllerIntegralLimit = ...
    p.driver.supplyVoltage/p.motor.speedControllerKi;
p.motor.speedCommandLimit = ...
    p.driver.maxDutyCycle*p.motor.noLoadSpeed;
p.motor.torquePerSpeedError = ...
    (p.driver.continuousTorqueLimit/p.driver.supplyVoltage)* ...
    p.motor.speedControllerKp;

% Legacy aliases keep archived model variants loadable during migration.
p.servo = p.motor;
p.servo.maxTorque = p.motor.stallTorque;
p.servo.maxSpeed = p.motor.noLoadSpeed;

% Redesigned triangular body. The body frame origin is the IMU location.
p.rover.bodyMass = 0.180;
p.rover.mass = p.rover.bodyMass + 3*(p.motor.mass + p.wheel.mass);
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
% Relaxed value used by the cascade model to reduce contact-state chatter.
p.contact.wheelBall.relaxedTransitionWidth = 1.5e-3;
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
% Limit the actuator command by the servo capability.  The former limit
% also clipped it to the nominal wheel-ball traction torque (about
% 0.042 N*m), which prevented the controller from developing recovery
% authority.  Slip and transmissible force remain governed by the
% Spatial Contact Force blocks in the plant.
p.wheel.commandTorqueLimit = p.driver.continuousTorqueLimit;
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
p.controller.maxYawRate = 0.80;
p.controller.recoveryTilt = deg2rad(18);
p.controller.fallenTilt = deg2rad(35);
p.controller.minimumContactConfidence = 0.20;
p.controller.recoveryGainScale = 1.35;

% Upright, centered, no-slip command limits. For pure translation the
% worst-case wheel rate is sin(lambda)*v/Rw. At half the resulting
% no-load-speed limit the FIT0521 torque-speed envelope still exceeds the
% wheel-ball traction limit, so traction limits acceleration, not speed.
p.controller.theoreticalMaxSpeed = p.wheel.radius* ...
    p.motor.speedCommandLimit/sin(p.wheel.contactLatitude);
p.controller.commandSpeedFraction = 0.50;
p.controller.targetCommandSpeed = p.controller.commandSpeedFraction* ...
    p.controller.theoreticalMaxSpeed;
p.controller.maxSpeed = p.controller.targetCommandSpeed;
p.controller.tractionLimitedWheelTorque = min( ...
    p.driver.continuousTorqueLimit, p.wheel.contactTorqueLimit);
p.controller.theoreticalTractionAcceleration = sqrt(3)* ...
    p.controller.tractionLimitedWheelTorque/(p.wheel.radius* ...
    (p.rover.mass + p.ball.mass));
p.controller.theoreticalLeanAcceleration = ...
    p.gravity*tan(p.controller.maxLean);
p.controller.theoreticalCommandAcceleration = min([ ...
    p.controller.maxPlanarAcceleration, ...
    p.controller.theoreticalTractionAcceleration, ...
    p.controller.theoreticalLeanAcceleration]);

% Ten-state MIMO LQI with the physical 55-degree three-wheel geometry.
rollingXY = p.wheel.geometry.rollingBody(1:2, :).';
p.controller.lqi.wheelSpeedFromPlanarVelocity = [ ...
    -(sin(p.wheel.contactLatitude)/p.wheel.radius)*rollingXY, ...
    (p.rover.chassisRadius/p.wheel.radius)*ones(3, 1)];
p.controller.lqi.maximumState = [p.controller.maxSpeed; ...
    p.controller.maxSpeed; p.controller.maxLean; p.controller.maxLean; ...
    1.5; 1.5; p.controller.maxYawRate; ...
    p.motor.speedCommandLimit*ones(3, 1)];
p.controller.lqi.maximumVelocityIntegral = [0.12; 0.12];
p.controller.lqi.maximumWheelSpeedCommand = ...
    0.70*p.motor.speedCommandLimit*ones(3, 1);
p.controller.lqi.linearizationStateStep = [1.0e-5; 1.0e-5; ...
    1.0e-6; 1.0e-6; 1.0e-5; 1.0e-5; 1.0e-5; ...
    1.0e-4; 1.0e-4; 1.0e-4];
p.controller.lqi.linearizationInputStep = 1.0e-4*ones(3, 1);
p.controller.lqi.antiWindupGain = 8.0;
p.controller.lqi.trackingGainScale = 1.0e-4;
p.controller.lqi = ballbotDesignLqi(p, p.controller.lqi);

% Upright PID and direct velocity-to-torque controller used by
% ball_balancing_omni3_multibody_pid.slx. The tilt reference is fixed at
% zero; commanded planar velocity contributes torque directly and does not
% generate a lean-angle reference.
p.controller.pidTiltKp = [0.95; 0.95];
p.controller.pidTiltKi = [0.02; 0.02];
p.controller.pidTiltKd = [0.12; 0.12];
p.controller.pidTiltIntegralLimit = deg2rad([5; 5]);
p.controller.pidVelocityKp = [0.03; 0.03];
p.controller.pidYawRateKp = 0.08;

p.command.velocityWorld = [p.controller.targetCommandSpeed; 0];
p.command.yawRate = 0;
p.command.enable = true;

p.simulation.stopTime = 4.0;
p.simulation.maxStep = 1.0e-4;
p.simulation.relativeTolerance = 1.0e-4;
p.simulation.imuYawBias = 0;
end
