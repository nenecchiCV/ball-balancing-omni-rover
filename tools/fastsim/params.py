"""Ported parameters from ballbotParameters.m / ballbotRev1Parameters.m.

Flat namespace P mirrors the MATLAB `p` struct fields actually used by the
rev1 torque-LQI path, the estimator, and the reduced 2-body plant.
"""
import numpy as np

G = 9.80665

# Ball / wheel / geometry ----------------------------------------------------
Rb = 0.050
Mb = 0.285
Ib_scalar = (2.0 / 3.0) * Mb * Rb**2          # thin shell
Rw = 0.024
wheel_azimuth = np.deg2rad([0.0, 120.0, 240.0])
contact_latitude = np.deg2rad(65.0)
contact_preload = 5.0e-5

# Motor (FIT0521, output-shaft equivalent) -----------------------------------
motor_nominal_voltage = 6.0
motor_gear_ratio = 34.02
motor_no_load_speed = 210 * 2 * np.pi / 60.0
motor_no_load_current = 0.13
motor_stall_torque = 10 * 9.80665 / 100.0      # 10 kgf*cm -> N*m = 0.9807
motor_stall_current = 3.2
motor_armature_R = motor_nominal_voltage / motor_stall_current   # 1.875
motor_Kt = motor_stall_torque / motor_stall_current              # 0.3065
motor_Ke = (motor_nominal_voltage - motor_no_load_current * motor_armature_R) \
    / motor_no_load_speed                                        # 0.2618
motor_b = motor_Kt * motor_no_load_current / motor_no_load_speed  # 0.00181
motor_tau = 0.030
motor_Jeq = motor_tau * motor_stall_torque / motor_no_load_speed  # 0.001338
# Armature electrical time constant (~L/R); FIT0521 has no datasheet L, use
# the typical small-motor figure matching the repo's "electrical scale" note.
motor_tau_e = 2.7e-3

battery_voltage = 7.2
max_duty = min(0.95, motor_nominal_voltage / battery_voltage)    # 0.8333
driver_continuous_current = 3.0
continuous_torque_limit = motor_Kt * min(driver_continuous_current,
                                         motor_stall_current)    # 0.919

# Rover lump (from ballbotParameters.m chassisParts table) ---------------------
rho = 1050.0
frame_disc_mass = rho * np.pi * 0.080**2 * 0.003
frame_disc_I = frame_disc_mass * (3 * 0.080**2 + 0.003**2) / 12
spacer_mass = rho * (3 * np.sqrt(3) / 2) * 0.003**2 * 0.050
spacer_I = spacer_mass * (3 * 0.003**2 + 0.050**2) / 12
mount_mass = rho * 44569.7e-9
strap_mass = rho * 2819.7e-9
motor_mass = 0.096
wheel_mass = 0.032
payload_length = 0.30
payload_width = 0.020
payload_mass_nominal = 0.50

chassis_parts = np.array([
    [frame_disc_mass, 0.0985,  frame_disc_I],
    [frame_disc_mass, 0.1515,  frame_disc_I],
    [frame_disc_mass, 0.2045,  frame_disc_I],
    [6 * spacer_mass, 0.12525, 6 * spacer_I],
    [6 * spacer_mass, 0.17825, 6 * spacer_I],
    [3 * mount_mass,  0.0602,  3 * mount_mass * 0.047**2 / 6],
    [3 * strap_mass,  0.0617,  3 * strap_mass * 0.02**2 / 12],
    [0.423,           0.1125,  0.423 * (0.140**2 + 0.025**2) / 12],
    [0.030,           0.103,   0.030 * (0.050**2 + 0.030**2) / 12],
    [0.090,           0.156,   0.090 * (0.133**2 + 0.080**2) / 12],
    [0.010,           0.110,   0.0],
])
body_mass = float(chassis_parts[:, 0].sum())
body_com = float((chassis_parts[:, 0] * chassis_parts[:, 1]).sum() / body_mass)
off_axis_yaw = 6 * spacer_mass * 0.065**2 * 2 + 3 * mount_mass * 0.061**2 \
    + 3 * strap_mass * 0.0768**2
body_tilt_I = float((chassis_parts[:, 2] + chassis_parts[:, 0]
                     * (chassis_parts[:, 1] - body_com)**2).sum()
                    + off_axis_yaw / 2)
body_I = np.array([body_tilt_I, body_tilt_I,
                   2 * float(chassis_parts[:, 2].sum()) + off_axis_yaw])
original_rover_mass = body_mass + 3 * (motor_mass + wheel_mass)
rover_center_above_ball = 0.125
rover_chassis_radius = 0.080


def payload_rover_props(payload_mass):
    """Exact port of ballbotPayloadRoverProperties.m."""
    wheel_z = (Rb + Rw - 4.0e-4) * np.sin(contact_latitude)
    base_com = (body_mass * body_com
                + 3 * (motor_mass + wheel_mass) * wheel_z) \
        / original_rover_mass
    payload_com = rover_center_above_ball + 0.030 / 2 + payload_length / 2
    mass = original_rover_mass + payload_mass
    com = (original_rover_mass * base_com + payload_mass * payload_com) / mass
    Icom = (body_I[0] + body_mass * (body_com - com)**2
            + 3 * (motor_mass * (3 * 0.0122**2 + 0.052**2) / 12
                   + motor_mass * (wheel_z - com)**2
                   + wheel_mass * Rw**2 / 2
                   + wheel_mass * (wheel_z - com)**2)
            + payload_mass * payload_length**2 / 12
            + payload_mass * (payload_com - com)**2)
    return mass, com, Icom


rover_mass, rover_com_above_ball, rover_tilt_inertia_com = \
    payload_rover_props(payload_mass_nominal)

# Inertia about ball pivot point
Ipivot = rover_tilt_inertia_com + rover_mass * rover_com_above_ball**2
Iyaw = body_I[2] + payload_mass_nominal * payload_width**2 / 12

gravity_gain = rover_mass * G * rover_com_above_ball / Ipivot      # gg
accel_gain = rover_mass * rover_com_above_ball / Ipivot

# Wheel geometry --------------------------------------------------------------
def wheel_geometry():
    beta = wheel_azimuth[None, :]
    lam = contact_latitude
    normal = np.vstack([np.cos(lam) * np.cos(beta),
                        np.cos(lam) * np.sin(beta),
                        np.full(3, np.sin(lam))])               # 3x3 body
    rolling = np.vstack([-np.sin(beta), np.cos(beta), np.zeros(3)])
    axle = np.cross(normal.T, rolling.T).T                      # 3x3 body
    cdist = Rb + Rw - contact_preload
    center_from_ball = cdist * normal
    body_from_ball = np.array([[0.0], [0.0], [rover_center_above_ball]])
    center_from_body = center_from_ball - body_from_ball
    return normal, rolling, axle, center_from_body

normalB, rollingB, axleB, centerFromBody = wheel_geometry()

# Kinematic wheel-rate map W: omega_w = W [vx vy r]^T  (controller convention)
rollingXY = rollingB[0:2, :].T                                   # 3x2
W_map = np.hstack([-(np.sin(contact_latitude) / Rw) * rollingXY,
                   (rover_chassis_radius / Rw) * np.ones((3, 1))])

# Normal load per wheel / traction budget -------------------------------------
# Measured assembled rest load is ~4.07 N/wheel (not mg/3sin(lat)); the
# rev1 clamp sits at 80% of the slip threshold and scales with supported
# mass relative to the nominal 0.5 kg build.
traction_friction = 1.0
normal_load_nominal = rover_mass * G / (3 * np.sin(contact_latitude))
mass_ratio = rover_mass / (original_rover_mass + 0.50)
contact_torque_limit = 0.80 * traction_friction * 4.07 * mass_ratio * Rw
traction_limited_torque = min(continuous_torque_limit, contact_torque_limit)

# Contact / ground ------------------------------------------------------------
rolling_friction = 0.008
rolling_crit_vel = 5.0e-2
spin_scrub_arm = 1.0e-3
spin_scrub_crit = 0.5

# torqueFit empirical scales --------------------------------------------------
force_scale = 0.5
tilt_scale = 0.425
yaw_scale = 0.0145
wheel_drive_matrix = 306.0 * np.array([
    [1, 0.106, 0.106], [0.106, 1, 0.105], [0.105, 0.106, 1]])
contact_coupling = 100.0 * motor_Jeq                              # N*m per rad/s

# Encoder ---------------------------------------------------------------------
encoder_ppr = 341.2
enc_quant = 2 * np.pi / encoder_ppr                               # rad/count

# Sample times ----------------------------------------------------------------
Ts = 0.005

# Estimator params ------------------------------------------------------------
est = dict(
    sampleTime=Ts,
    attitudeCorrectionGain=2.5,
    accelNormGate=0.25 * G,
    accelHorizontalGate=0.03,
    velocityLeak=0.9995,
    ballRateTimeConstant=0.080,
    relativePositionLeak=0.9967,
    velocityAnchorTimeConstant=0.5,
    velocityAnchorMinSpeed=0.05,
    stillnessFilterTimeConstant=0.15,
    ballRateStillWheelRateFiltered=0.8,
    stillVelocityDrainTimeConstant=0.1,
    stillVelocityDrainHoldTime=1.0,
    ballRateStillGyroRate=0.15,
    relativePositionLimit=0.12,
    kinematicRegularization=1.0e-8,
    biasWheelRateThreshold=0.10,
    biasAccelNormThreshold=0.03 * G,
    biasRollPitchRateThreshold=0.02,
    biasYawRateThreshold=0.05,
    biasQualificationTime=0.50,
    biasQualificationHysteresis=1.25,
    biasTimeConstant=1.0,
    biasMaximum=0.10,
    biasMaximumUpdatePerSample=1.0e-4,
)

# Controller params -----------------------------------------------------------
slip_limit_wheel_speed = motor_no_load_speed * (
    1 - contact_torque_limit / motor_stall_torque)
slip_limit_speed = min(
    slip_limit_wheel_speed * Rw / np.sin(contact_latitude),
    Rw * max_duty * motor_no_load_speed / np.sin(contact_latitude))

ctrl = dict(
    sampleTime=Ts,
    maxSpeed=slip_limit_speed,
    maxYawRate=0.80,
    maxPlanarAcceleration=0.60,
    recoveryTilt=np.deg2rad(18),
    fallenTilt=np.deg2rad(35),
    recoveryGainScale=1.35,
    tiltPriorityStart=np.deg2rad(3),
    velocityIntegralLimit=np.array([0.20, 0.20]),
    yawBiasCommandBypassThreshold=1.0e-6,
    yawBiasReadyRateThreshold=0.002,
    coupledCommandLimitEnabled=True,
    speedCommandLimit=max_duty * motor_no_load_speed,
)

td = dict(  # torqueDrive
    enabled=True,
    slipDampingGain=0.02,
    slipDeadzone=1.2 * enc_quant / Ts,
    backEmfSpeedTimeConstant=1.0e-3,
    rateFilterTimeConstant=0.02,
    wheelRateFilterTimeConstant=0.03,
    filteredSlipDeadzone=1.0,
    integralLeak=2.0,
    integralEnabled=False,
    velocityLeanGain=0.3,
    leanSetpointLimit=0.009,
    accelFeedforwardGain=1.0,
    leanFadeTilt=0.02,
    leanRateLimit=0.05,
    tiltStiffenGain=2.0,
    # Disabled (measured on the Simulink plant): driveGain=306 predicts
    # the free-wheel rate response (~100x faster than the loaded one),
    # inflating the rate estimate during any sustained torque -- kills
    # the propulsion error and raises slipRate into phantom braking.
    # See ballbotRev1Parameters.m for the measured stall numbers.
    wheelRateObserverEnabled=False,
    wheelRateObserverGain=0.15,
    wheelRateObserverDriveGain=306.0,
    wheelRateObserverCompliance=0.134,
    tiltStiffenTilt=0.006,
    tiltIntegralGain=1.0,
    tiltIntegralLimit=0.03,
    tiltIntegralLeak=0.5,
    startupTorqueRampTime=0.4,
    propulsionGain=0.02,
    propulsionTorqueLimit=0.045,
    propulsionMinShare=0.25,
    stopFollowGain=0.75,
    ballYawGain=0.12,
    ballYawTorqueLimit=0.025,
    yawDampingGain=0.30,
    yawGate=dict(maxSpeed=0.03, maxTilt=0.02, commandThreshold=1.0e-3),
    wheelRateFeedbackEnabled=False,
)

lqi_antiwindup = 8.0

# Command profile -------------------------------------------------------------
cmd = dict(
    cyclesEnabled=True,
    cycleStart=0.5,
    cycleMoveDuration=12.0,
    cycleStopDuration=4.0,
    cycleYawDelay=0.0,
    # Verified safe on v12 (obs-off): 0.2 rad/s x 1.0 s trapezoid
    # (~0.1 rad) at the start of the stop window, thinned to every
    # other stop via cycleYawEvery=2, completes 49.5 s.  Denser or
    # larger bursts all tip (see ballbotRev1Parameters.m).
    cycleYawDuration=1.0,
    # Thins the yaw window to every cycleYawEvery-th stop phase (1 =
    # every stop window, 0 disables thinning), same as MATLAB.
    cycleYawEvery=2,
    cycleCount=3,
    accelerationLimit=0.03,
    decelerationLimit=0.01,
    yawPulseDuration=0.0,
    profileVelocityWorld=np.array([0.12, 0.0]),
    profileYawRate=0.20,
)

# Ground friction bound on the escape (centering) force: the ball cannot
# be pushed out from under the rover harder than the ground contact's
# tangential grip allows.  Rubber-sprayed ball on floor: ~0.4 effective.
escape_mu = 0.4
