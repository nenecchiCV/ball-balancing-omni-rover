"""Two-body reduced plant for the rev1 ballbot (fitted-measurement version).

Reuses the fitted open-loop response the rev1 gains were designed on
(measuredB0 torque-step matrix, W'τ tangential drive map, fitted wheel
drive matrix), but splits translation into separate rover and ball DOF so
the relPos/escape mode exists:

  states x = [pr(2); vr(2); tilt(2); dtilt(2); psi; r; pb(2); vb(2);
              obz; w(3); i_arm(3); thw(3)]
  tilt = [pitch; -roll], dtilt = [q; -p]  (controller sign convention)

  * Drive: tangential contact force F_t = forceScale*(W'τ)(1:2) acts + on
    the ball and - on the rover (action-reaction pair); the un-modelled
    share is contact/ground loss (external sink).
  * Gravity-through-offset-contact spring C*(pr-pb): restoring on rover,
    escaping on ball — the runaway/escape mechanism.
  * Pendulum tilt: gg*θ - mgh/I * a_ball_support + measured-B tilt rows.
  * Wheels: fitted wheelDriveMatrix*(τ - c(w - wkin)); wkin from the full
    rolling constraint (estimator kinematics).
  * Ball z-spin: (Rb/Rw)*axle_z torque map (ballTorqueFromWheelTorque) +
    ground scrub.
  * Motor: duty inversion + armature lag, duty clipped at maxDuty.
"""
import numpy as np
import params as P
import design as D

NX = 24
IPR, IVR, ITILT, IDTILT, IPSI, IR, IPB, IVB, IOBZ, IW, IIARM, ITHW = (
    slice(0, 2), slice(2, 4), slice(4, 6), slice(6, 8), 8, 9,
    slice(10, 12), slice(12, 14), 14, slice(15, 18), slice(18, 21),
    slice(21, 24))

Mb_eff = P.Mb + P.Ib_scalar / P.Rb**2
C_center = P.rover_mass * P.G / (P.Rb + P.Rw)
D_esc = 0.0
N_load = (P.rover_mass + P.Mb) * P.G

# Measured-B rows for the CURRENT build (tilt/yaw scaled by inertia ratio).
_B = D.MEASURED_B0.copy()
_B[0:2, :] *= D.IPIVOT_REF / P.Ipivot
_B[2, :] *= D.IYAW_REF / P.Iyaw
B_TILT = np.vstack([_B[0, :], -_B[1, :]])          # [pitch; -roll] accel
B_YAW = _B[2, :]
# Contact force on the ball is -f t_i; its z-moment is -f*Rb*axle_z.
BALL_SPIN_MAP = -(P.Rb / P.Rw) * P.axleB[2, :]


def wheel_kinematic_rate(x):
    """Kinematic (no-slip) wheel rates via the estimator's constraint."""
    vr, dtilt, r = x[IVR], x[IDTILT], x[IR]
    vb, obz = x[IVB], x[IOBZ]
    omr = np.array([-dtilt[1], dtilt[0], r])
    omb = np.array([-vb[1] / P.Rb, vb[0] / P.Rb, obz])
    out = np.empty(3)
    for i in range(3):
        t = P.rollingB[:, i]
        n = P.normalB[:, i]
        c = P.centerFromBody[:, i]
        v_ball = np.array([vb[0], vb[1], 0.0]) + np.cross(omb, P.Rb * n)
        v_wheel = np.array([vr[0], vr[1], 0.0]) + np.cross(omr, c)
        # No-slip: wheel contact surface v_wc - w*Rw*t_i == ball surface.
        out[i] = np.dot(t, v_wheel - v_ball) / P.Rw
    return out


def plant_deriv(x, tau_motor):
    vr, tilt, dtilt = x[IVR], x[ITILT], x[IDTILT]
    r, vb, obz = x[IR], x[IVB], x[IOBZ]
    w = x[IW]
    relPos = x[IPR] - x[IPB]

    wkin = wheel_kinematic_rate(x)
    # Tangential drive force on the ball (fitted), reaction on the rover.
    # The wheel-ball rolling constraint adds the reaction of the slip
    # drag: contact torque on the wheel -c*(w - wkin) has an equal
    # tangential force +c*(w - wkin)/Rw on the ball along t_i (wheels
    # roll over the ball; the ball cannot translate without turning the
    # wheels).  On the rover the same pair applies with opposite sign.
    # Wheel tangential directions t_i = -row_xy(W_map) normalized
    # (W_map row i is -t_i/Rw against ball velocity by construction).
    T_mat = -P.W_map[:, 0:2].T.copy()          # 2 x 3, columns ~ -t_i/Rw
    for j in range(3):
        n = np.linalg.norm(T_mat[:, j])
        if n > 0:
            T_mat[:, j] /= n
    # Slip contact force, saturated at the wheel traction budget
    # (same physics as the torque clamp): a real contact cannot drag
    # the ball harder than mu*N tangentially.
    slip_vec = -(P.contact_coupling / P.Rw) * (w - wkin)
    slip_max = P.traction_friction * 4.07 * P.mass_ratio
    slip_vec = np.clip(slip_vec, -slip_max, slip_max)
    Fslip = T_mat @ slip_vec
    Ft = P.force_scale * (P.W_map.T @ tau_motor)[0:2] + Fslip
    vbn = np.linalg.norm(vb)
    # Smooth rolling drag: the tanh/vc form gives a ~200x slope near
    # v=0 that turns support-acceleration coupling into a strong
    # destabilizing velocity->tilt gain.  Rate-proportional below the
    # knee is closer to continuous contact friction and keeps the
    # Jacobian sane.
    Frr = -P.rolling_friction * N_load * vb / (vbn + P.rolling_crit_vel)
    # Escape force: gravity through the offset contacts squeezes the ball
    # out the BACK of the lean (ball gets -relPos) while the tilted
    # contact normals push the rover further along +relPos -- the
    # divergent separation mode.  Contact/rolling losses damp it
    # (D_esc).  It cannot exceed what the ball-ground contact transmits --
    # clip at the effective ground-friction budget.
    relVel = vr - vb
    Fesc = C_center * relPos - D_esc * relVel
    esc_max = P.escape_mu * N_load
    esc_n = np.linalg.norm(Fesc)
    if esc_n > esc_max:
        Fesc = Fesc * (esc_max / esc_n)
    ab = (Ft - Fesc + Frr) / Mb_eff
    ar = (-Ft + Fesc) / P.rover_mass

    # Pendulum support acceleration: the torque->tilt response is already
    # inside the measured-B rows, so only the NON-drive share of ball
    # acceleration (centering spring + rolling drag) couples here.
    ab_support = (-Fesc + Frr) / Mb_eff
    dtilt2 = (P.gravity_gain * tilt - P.accel_gain * ab_support
              + B_TILT @ tau_motor)
    dr = B_YAW @ tau_motor

    scrub = -P.spin_scrub_arm * N_load * np.tanh(obz / P.spin_scrub_crit)
    dobz = (P.force_scale * (BALL_SPIN_MAP @ tau_motor) / P.Ib_scalar
            + scrub / P.Ib_scalar)

    dw = P.wheel_drive_matrix @ (tau_motor
                                 - P.contact_coupling * (w - wkin))
    return np.concatenate([
        vr, ar, dtilt, dtilt2, [r, dr], vb, ab, [dobz], dw,
        np.zeros(3), w])


def motor_step(w, i_arm, tau_cmd, dt):
    """One control-period motor update; returns delivered torque per wheel."""
    global _w_f
    try:
        _w_f
    except NameError:
        _w_f = w.copy()
    a = dt / (P.td["backEmfSpeedTimeConstant"] + dt)
    _w_f = _w_f + a * (w - _w_f)
    Vs = P.battery_voltage
    duty = (P.motor_armature_R * (tau_cmd / P.motor_Kt)
            + P.motor_Ke * _w_f) / Vs
    duty = np.clip(duty, -P.max_duty, P.max_duty)
    L = P.motor_tau_e * P.motor_armature_R
    n_sub = 8
    h = dt / n_sub
    i = i_arm.copy()
    for _ in range(n_sub):
        i = i + h * (Vs * duty - P.motor_Ke * w - i * P.motor_armature_R) / L
    tau = P.motor_Kt * i - P.motor_b * w
    return tau, i, duty


def rot_world_from_body(tilt, psi):
    roll, pitch, yaw = -tilt[1], tilt[0], psi
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch),
                             np.sin(pitch), np.cos(yaw), np.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def imu_sample(x, ar, tilt_rate):
    """Ideal IMU: specific force (body) and gyro (body)."""
    R = rot_world_from_body(x[ITILT], x[IPSI])
    a_world = np.array([ar[0], ar[1], 0.0])
    spec = R.T @ (a_world - np.array([0.0, 0.0, -P.G]))
    gyro_world = np.array([-tilt_rate[1], tilt_rate[0], x[IR]])
    gyro = R.T @ gyro_world
    return np.concatenate([spec, gyro])


class Encoder:
    """341.2 PPR quadrature counter -> per-sample backward-difference rate."""

    def __init__(self):
        self.prev_counts = np.zeros(3, dtype=np.int64)

    def counts(self, thw):
        return np.round(thw / P.enc_quant).astype(np.int64)

    def rate(self, thw, dt):
        c = self.counts(thw)
        rate = (c - self.prev_counts) * P.enc_quant / dt
        self.prev_counts = c
        return rate


def initial_state(vb0=None, tilt0=0.0):
    x = np.zeros(NX)
    x[ITILT] = tilt0
    if vb0 is not None:
        x[IVB] = vb0
        x[IVR] = vb0
    return x
