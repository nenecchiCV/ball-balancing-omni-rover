"""Port of ballbotEstimatorStep.m (14-state IMU/wheel observer).

State: [q_WB(4); v_WB(3); omegaBall_W(3); roverMinusBall_B_xy(2);
        gyroBiasZ; qualificationTime].
Estimate: [roll; pitch; yaw; p; q; r; vx_W; vy_W; omegaBall_W(3);
           relPosB_xy(2)].
"""
import numpy as np
import params as P
import model as M


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _q_to_R(q):
    q = q / max(np.linalg.norm(q), 1e-300)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _integrate_q(q, rate, dt):
    Om = np.array([
        [0, -rate[0], -rate[1], -rate[2]],
        [rate[0], 0, rate[2], -rate[1]],
        [rate[1], -rate[2], 0, rate[0]],
        [rate[2], rate[1], -rate[0], 0]])
    q = q + 0.5 * dt * Om @ q
    return q / max(np.linalg.norm(q), 1e-300)


def _R_to_euler(R):
    pitch = np.arcsin(min(max(-R[2, 0], -1), 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


class Estimator:
    def __init__(self):
        e = P.est
        self.state = np.zeros(14)
        self.state[0] = 1.0
        self.filtered_wheel_rate = np.zeros(3)
        self.stillness_hold = 0.0

    def step(self, imu, wheel_rate):
        e = P.est
        dt = e["sampleTime"]
        s = self.state
        quaternion = s[0:4].copy()
        velocityWorld = s[4:7].copy()
        ballRatePrev = s[7:10].copy()
        relPosBodyPrev = s[10:12].copy()
        gyroBiasZ = s[12]
        qualificationTime = s[13]

        spec = imu[0:3]
        gyro_raw = imu[3:6]
        gyro = gyro_raw - np.array([0.0, 0.0, gyroBiasZ])
        R = _q_to_R(quaternion)

        acc_norm = np.linalg.norm(spec)
        quasiStatic = (np.max(np.abs(wheel_rate))
                       <= e["biasWheelRateThreshold"]
                       and np.hypot(gyro[0], gyro[1])
                       <= e["biasRollPitchRateThreshold"]
                       and abs(gyro[2]) <= e["biasYawRateThreshold"])
        if abs(acc_norm - P.G) <= e["accelNormGate"] and quasiStatic:
            up_meas = spec / max(acc_norm, 1e-300)
            up_pred = R.T @ np.array([0.0, 0.0, 1.0])
            corr = e["attitudeCorrectionGain"] * np.cross(up_meas, up_pred)
            m = np.linalg.norm(corr)
            if m > 0.5:
                corr = corr * (0.5 / m)
        else:
            corr = np.zeros(3)

        quaternion = _integrate_q(quaternion, gyro + corr, dt)
        R = _q_to_R(quaternion)

        accW = R @ spec + np.array([0.0, 0.0, -P.G])
        velocityWorld = e["velocityLeak"] * velocityWorld + dt * accW
        velocityWorld[2] = 0.0

        normalW = R @ P.normalB
        rollingW = R @ P.rollingB
        wheelCenterW = R @ P.centerFromBody
        bodyRateW = R @ gyro

        GRM = np.array([[0, P.Rb, 0], [-P.Rb, 0, 0], [0, 0, 0]])
        K = np.zeros((3, 3))
        rhs = np.zeros(3)
        for i in range(3):
            S = GRM - P.Rb * _skew(normalW[:, i])
            K[i, :] = rollingW[:, i] @ S
            wcv = velocityWorld + np.cross(bodyRateW, wheelCenterW[:, i])
            rhs[i] = rollingW[:, i] @ wcv - P.Rw * wheel_rate[i]

        ballRateKin = np.linalg.solve(
            K.T @ K + e["kinematicRegularization"] * np.eye(3), K.T @ rhs)

        wStill = dt / (e["stillnessFilterTimeConstant"] + dt)
        self.filtered_wheel_rate += wStill * (
            wheel_rate - self.filtered_wheel_rate)
        assemblyStill = (np.max(np.abs(self.filtered_wheel_rate))
                         <= e["ballRateStillWheelRateFiltered"]
                         and np.linalg.norm(gyro)
                         <= e["ballRateStillGyroRate"])
        self.stillness_hold = (self.stillness_hold + dt) if assemblyStill else 0.0
        if self.stillness_hold >= e["stillVelocityDrainHoldTime"]:
            ballRateKin[:] = 0.0

        wBall = dt / (e["ballRateTimeConstant"] + dt)
        ballRate = ballRatePrev + wBall * (ballRateKin - ballRatePrev)
        ballVel = GRM @ ballRate

        if self.stillness_hold >= e["stillVelocityDrainHoldTime"]:
            tau = e["stillVelocityDrainTimeConstant"]
            velocityWorld[0:2] *= tau / (tau + dt)
        elif (e["velocityAnchorTimeConstant"] > 0
              and np.linalg.norm(ballVel) > e["velocityAnchorMinSpeed"]):
            w = dt / (e["velocityAnchorTimeConstant"] + dt)
            velocityWorld[0:2] += w * (ballVel[0:2] - velocityWorld[0:2])

        relVelW = velocityWorld - ballVel
        euler = _R_to_euler(R)
        cy, sy = np.cos(euler[2]), np.sin(euler[2])
        Ryaw = np.array([[cy, -sy], [sy, cy]])
        relPosWPrev = Ryaw @ relPosBodyPrev
        relPosW = e["relativePositionLeak"] * relPosWPrev \
            + dt * relVelW[0:2]
        n = np.linalg.norm(relPosW)
        if n > e["relativePositionLimit"]:
            relPosW *= e["relativePositionLimit"] / n
        relPosBody = Ryaw.T @ relPosW

        qualifiedPrev = qualificationTime >= e["biasQualificationTime"]
        thr = e["biasQualificationHysteresis"] if qualifiedPrev else 1.0
        cand = (np.max(np.abs(wheel_rate))
                <= thr * e["biasWheelRateThreshold"]
                and abs(acc_norm - P.G)
                <= thr * e["biasAccelNormThreshold"]
                and np.hypot(gyro[0], gyro[1])
                <= thr * e["biasRollPitchRateThreshold"]
                and abs(gyro[2]) <= thr * e["biasYawRateThreshold"])
        qualificationTime = (min(qualificationTime + dt,
                                 e["biasQualificationTime"]) if cand else 0.0)

        learn = cand and qualifiedPrev
        upd = float(learn) * (dt / (e["biasTimeConstant"] + dt)) \
            * (gyro_raw[2] - gyroBiasZ)
        upd = min(max(upd, -e["biasMaximumUpdatePerSample"]),
                  e["biasMaximumUpdatePerSample"])
        gyroBiasZ = min(max(gyroBiasZ + upd, -e["biasMaximum"]),
                        e["biasMaximum"])

        self.state = np.concatenate([
            quaternion, velocityWorld, ballRate, relPosBody,
            [gyroBiasZ, qualificationTime]])
        estimate = np.concatenate([
            euler, gyro, velocityWorld[0:2], ballRate, relPosBody])
        return estimate
