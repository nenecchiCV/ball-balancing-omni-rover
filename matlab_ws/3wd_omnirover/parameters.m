% 3WD omnirover physical and control parameters.

wheelRadius = 0.024;                 % m, Nexus 14108 diameter 48 mm
wheelWidth = 0.0251;                 % m
wheelMass = 0.039;                   % kg
wheelRollerCount = 8;

servoDimensions = [0.0417 0.0197 0.0429]; % m
servoMass = 0.055;                   % kg
servoMaxTorque = 13 * 9.80665 / 100; % N*m, 13 kgf*cm
servoMinSpeed = 53 * 2*pi / 60;      % rad/s
servoMaxSpeed = 62 * 2*pi / 60;      % rad/s
servoNominalVoltage = 5;             % V, rover operating voltage
servoOperatingCurrent = 0.100;       % A

roverDiameter = 0.160;               % m
roverHeight = 0.105;                 % m
roverMass = 0.462;                   % kg, including battery
wheelCenterRadius = roverDiameter/2 - wheelRadius;
chassisHeight = 0.030;               % m, simplified frame
chassisDimensions = [0.120 0.120 chassisHeight];
chassisMass = roverMass - 3*(servoMass + wheelMass);

wheelAngles = deg2rad([0 120 240]);   % rad, wheel radial locations
wheelLongitudinalFriction = 1.0;
wheelLateralFriction = 0.0;           % ideal omni rollers
% Custom Tire block force direction is tire-on-ground; the negative gain
% converts the desired ground-on-tire feedback force to that convention.
wheelSpeedGain = -25;                 % N/(m/s), contact speed servo gain
wheelForceLimit = servoMaxTorque / wheelRadius;

% Lonrium-equivalent vinyl floor contact model. The friction coefficient is
% exposed for calibration because the actual value depends on the roller
% compound, surface finish, dust, and temperature.
floorLongitudinalStaticFriction = 0.75;
floorLongitudinalDynamicFriction = 0.65;
floorLateralFriction = 0.0;           % free omni-roller direction
floorNormalStiffness = 1.5e5;         % N/m
floorNormalDamping = 450;             % N/(m/s)
floorDeflectionFilterTime = 2e-3;     % s
floorNominalLoadPerWheel = roverMass*gravityConstant()/3;
floorNominalDeflection = floorNominalLoadPerWheel/floorNormalStiffness;

commandVxWorld = 0.12;               % m/s
commandVyWorld = 0.06;               % m/s
commandYawRateWorld = 0.35;           % rad/s
simulationStopTime = 6;               % s

function value = gravityConstant
value = 9.80665;
end
