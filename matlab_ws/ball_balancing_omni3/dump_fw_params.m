% ファームウェア移植用パラメータダンプ（一時スクリプト）
p = ballbotRev1Parameters();
s = struct();
s.gravity = p.gravity;
s.ball_radius = p.ball.radius;
s.ball_mass = p.ball.mass;
s.ball_inertia_11 = p.ball.inertia(1, 1);
s.wheel_radius = p.wheel.radius;
s.wheel_radius_effective = p.wheel.radiusEffective;
s.wheel_azimuth = p.wheel.azimuth(:).';
s.contact_latitude = p.wheel.contactLatitude;
s.contact_preload = p.wheel.contactPreload;
s.motor_torque_constant = p.motor.torqueConstant;
s.motor_back_emf = p.motor.backEmfConstant;
s.motor_armature_resistance = p.motor.armatureResistance;
s.motor_no_load_speed = p.motor.noLoadSpeed;
s.motor_speed_command_limit = p.motor.speedCommandLimit;
s.driver_supply_voltage = p.driver.supplyVoltage;
s.driver_max_duty = p.driver.maxDutyCycle;
s.driver_continuous_torque_limit = p.driver.continuousTorqueLimit;
s.driver_bridge_on_resistance = p.driver.bridgeOnResistance;
s.encoder_ppr = p.encoder.pulsesPerOutputRevolution;
s.encoder_quantum = p.encoder.quantization;
s.encoder_quadrature = p.encoder.quadratureFactor;
s.rover_mass = p.rover.mass;
s.rover_center_above_ball = p.rover.centerAboveBall;
s.rover_com_above_ball = p.rover.comAboveBall;
s.rover_chassis_radius = p.rover.chassisRadius;
s.rover_body_inertia = p.rover.bodyInertia(:).';
s.rover_tilt_inertia_com = p.rover.tiltInertiaCom;
s.normal_load_nominal = p.wheel.normalLoadNominal;
s.contact_torque_limit = p.wheel.contactTorqueLimit;
s.rolling_friction = p.contact.ballGround.rollingFriction;
s.geom_normalBody = p.wheel.geometry.normalBody;
s.geom_rollingBody = p.wheel.geometry.rollingBody;
s.geom_axleBody = p.wheel.geometry.axleBody;
s.geom_centerFromBall = p.wheel.geometry.centerFromBall;
s.geom_centerFromBody = p.wheel.geometry.centerFromBody;
s.W_map = p.controller.lqi.wheelSpeedFromPlanarVelocity;
s.K_torque = p.controller.torqueLqi.stateGain;
s.K_integral = p.controller.torqueLqi.integralGain;
s.leanHoldTorque = p.controller.torqueDrive.leanHoldTorque;
s.traction_limited_torque = p.controller.tractionLimitedWheelTorque;
s.recovery_wheel_torque_limit = p.controller.recoveryWheelTorqueLimit;
s.estimator = jsafe(p.estimator);
s.controller = jsafe(p.controller);
s.command = jsafe(p.command);
s.drive = jsafe(p.drive);
j = jsonencode(s);
fid = fopen( ...
    'C:/Users/cafdr/Documents/GitHub/ball-balancing-omni-rover/firmware_ws/ballbot_params_dump.json', ...
    'w');
fwrite(fid, j);
fclose(fid);
fprintf('size=%d\n', numel(j));

function y = jsafe(x)
if isnumeric(x)
    if ~isreal(x)
        y = real(x);
    else
        y = double(x);
    end
elseif islogical(x)
    y = x;
elseif ischar(x) || isstring(x)
    y = char(x);
elseif isstruct(x)
    y = struct();
    f = fieldnames(x);
    for i = 1:numel(f)
        y.(f{i}) = jsafe(x.(f{i}));
    end
else
    y = 'UNSUPPORTED';
end
end
