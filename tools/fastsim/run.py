"""Closed-loop fast-sim runner: plant + encoder + estimator + controller.

Plant integrates at h = Ts/n_sub with the delivered torque held per
control tick. Sensor and controller cadence = 5 ms (p.estimator.sampleTime).
"""
import numpy as np
import params as P
import model as M
from estimator import Estimator
from controller import TorqueLqiController
import wheelspeed
import design


def command_profile(t, p_cmd=None):
    c = P.cmd if p_cmd is None else p_cmd
    cmd = np.array([0.0, 0.0, c.get("yawRate", 0.0)])
    accel = c["accelerationLimit"]
    if c.get("cyclesEnabled", False):
        target = np.asarray(c["profileVelocityWorld"], float)
        yaw_t = c.get("profileYawRate", 0.0)
        speed = np.linalg.norm(target)
        decel = c.get("decelerationLimit", accel)
        rampUp = min(speed / accel if accel > 0 else 0,
                     c["cycleMoveDuration"] / 2)
        rampDn = min(speed / decel if decel > 0 else 0,
                     c["cycleMoveDuration"] / 2)
        if speed <= 0 and abs(yaw_t) > 0:
            rampUp = rampDn = min(abs(yaw_t) / accel,
                                  c["cycleMoveDuration"] / 2)
        period = c["cycleMoveDuration"] + c["cycleStopDuration"]
        tEnd = c["cycleStart"] + c["cycleCount"] * period
        cmd[0:2] = 0.0
        cmd[2] = 0.0
        if c["cycleStart"] <= t < tEnd and (rampUp > 0 or rampDn > 0):
            tc = (t - c["cycleStart"]) % period
            moveEnd = c["cycleMoveDuration"]
            if tc < rampUp:
                fraction = tc / rampUp
            elif tc < moveEnd - rampDn:
                fraction = 1.0
            elif tc < moveEnd:
                fraction = (moveEnd - tc) / rampDn
            else:
                fraction = 0.0
            cmd[0:2] = fraction * target
            # Yaw is time-separated from translation (the common-mode
            # torque that yaws the chassis also spins the ball, so doing
            # both at once drains the tilt loop's torque budget).  The
            # yaw trapezoid sits inside the stop window after
            # cycleYawDelay, same as ballbotRev1CommandProfile.
            yawStart = moveEnd + c.get("cycleYawDelay", 0.0)
            yawDur = c.get("cycleYawDuration", 0.0)
            yawEnd = yawStart + yawDur
            yawEvery = c.get("cycleYawEvery", 0)
            yawAllowed = (yawEvery <= 0 or
                          int((t - c["cycleStart"]) // period)
                          % yawEvery == 0)
            if yawAllowed and yawStart <= tc < yawEnd and yawDur > 0:
                yawRamp = min(abs(yaw_t) / accel if accel > 0 else 0,
                              yawDur / 2)
                yp = tc - yawStart
                if yp < yawRamp:
                    yf = yp / yawRamp
                elif yp < yawDur - yawRamp:
                    yf = 1.0
                else:
                    yf = (yawDur - yp) / yawRamp
                cmd[2] = yaw_t * yf
    else:
        v = np.asarray(c.get("velocityWorld", [0.0, 0.0]), float)
        cmd[0:2] = np.clip(t * accel, 0, 1) * v if t > 0 else [0, 0]
        cmd[2] = c.get("yawRate", 0.0)
    # Bounded yaw pulse: sustained common-mode yaw pumps the ball's
    # free z-spin and the plant loses a wheel ~0.4-0.6 rad in, so demo
    # scenarios cap the rotation instead of holding a rate (0 = off).
    # Non-cycle path only — the cycle profile bounds yaw via its own
    # stop-window trapezoid.
    dur = c.get("yawPulseDuration", 0.0)
    if dur > 0 and cmd[2] != 0 and not c.get("cyclesEnabled", False):
        ramp = min(abs(cmd[2]) / accel if accel > 0 else 0, dur / 2)
        if t < ramp:
            f = t / ramp
        elif t < dur - ramp:
            f = 1.0
        elif t < dur:
            f = (dur - t) / ramp
        else:
            f = 0.0
        cmd[2] *= f
    return cmd


def run(t_final=60.0, speed=0.306, n_cycles=3, wheel_meas="bdiff",
        meas_kw=None, gain=None, K8=None, tilt0=0.002,
        move_dur=12.0, stop_dur=4.0, seed=None, td_overrides=None,
        imu_noise=None, n_sub=8, log=True):
    """Run the cycle test; returns dict of logs/metrics.

    gain: (K10, leanHold) or None -> design via design.design_state_gain().
    K8:   alternative: raw 8-state gain to expand like the .m path.
    """
    if gain is None:
        K10, info = design.design_state_gain()
        leanHold = info["leanHold"]
    else:
        K10, leanHold = gain
    if K8 is not None:
        K10 = np.zeros((3, 10))
        K10[:, 2:4] = 1.8 * K8[:, 0:2]
        K10[:, 4:6] = 1.0 * K8[:, 2:4]
        K10[:, 6] = 5.0 * K8[:, 5]

    td_backup = {}
    if td_overrides:
        for k, v in td_overrides.items():
            td_backup[k] = P.td.get(k)
            P.td[k] = v

    p_cmd = dict(P.cmd)
    p_cmd["profileVelocityWorld"] = np.array([speed, 0.0])
    p_cmd["cycleCount"] = n_cycles
    p_cmd["cycleMoveDuration"] = move_dur
    p_cmd["cycleStopDuration"] = stop_dur
    p_cmd["profileYawRate"] = 0.0     # translation-only by default

    x = M.initial_state(tilt0=tilt0)
    i_arm = np.zeros(3)
    tau_del = np.zeros(3)
    est = Estimator()
    ctl = TorqueLqiController(K10, np.zeros((3, 2)), leanHold)
    enc = wheelspeed.make(wheel_meas, **(meas_kw or {}))

    rng = np.random.default_rng(seed)
    dt = P.Ts
    h = dt / n_sub
    n_steps = int(t_final / dt)
    logs = {k: np.zeros((n_steps, 3)) for k in
            ["tilt", "tau_cmd", "w_meas", "w_true"]}
    logs.update({k: np.zeros((n_steps, 2)) for k in
                 ["vr", "vb", "relPos", "est_vball", "cmd_v"]})
    logs.update({k: np.zeros(n_steps) for k in
                 ["obz", "psi", "r", "mode", "contactConf", "sat",
                  "est_vr_x"]})
    est_log = np.zeros((n_steps, 14))

    failed_at = None
    for k in range(n_steps):
        t = k * dt
        # --- sensors ---
        ar_now = M.plant_deriv(x, tau_del)[M.IVR]
        spec_gyro = M.imu_sample(x, ar_now, x[M.IDTILT])
        if imu_noise:
            spec_gyro = spec_gyro + np.concatenate([
                rng.normal(0, imu_noise[0], 3),
                rng.normal(0, imu_noise[1], 3)])
        tau_prev = ctl.prev_torque_command
        wkin_est = P.W_map @ np.array([est.state[4], est.state[5], 0.0])
        w_meas = enc.rate(x[M.ITHW], dt, tau_cmd=tau_prev, wkin=wkin_est)
        estimate = est.step(spec_gyro, w_meas)
        cmd = command_profile(t, p_cmd)
        tau_cmd, mode, _ = ctl.step(estimate, cmd, True, w_meas)

        # --- actuators + plant ---
        tau_del, i_arm, _duty = M.motor_step(x[M.IW], i_arm, tau_cmd, dt)
        ar_last = np.zeros(2)
        for _ in range(n_sub):
            dx = M.plant_deriv(x, tau_del)
            ar_last = dx[M.IVR]
            x = x + h * dx

        if log:
            estimate_now = estimate
            logs["tilt"][k] = [estimate_now[0], estimate_now[1],
                               np.hypot(estimate_now[0], estimate_now[1])]
            logs["tau_cmd"][k] = tau_cmd
            logs["w_meas"][k] = w_meas
            logs["w_true"][k] = x[M.IW]
            logs["vr"][k] = x[M.IVR]
            logs["vb"][k] = x[M.IVB]
            logs["relPos"][k] = x[M.IPR] - x[M.IPB]
            logs["est_vball"][k] = [P.Rb * estimate_now[9],
                                    -P.Rb * estimate_now[8]]
            logs["cmd_v"][k] = cmd[0:2]
            logs["obz"][k] = x[M.IOBZ]
            logs["psi"][k] = x[M.IPSI]
            logs["r"][k] = x[M.IR]
            logs["mode"][k] = mode
            logs["contactConf"][k] = estimate_now[13]
            logs["sat"][k] = float(np.any(
                np.abs(tau_cmd) >= P.traction_limited_torque - 1e-9))
            logs["est_vr_x"][k] = estimate_now[6]
            est_log[k] = estimate_now
        if np.hypot(estimate[0], estimate[1]) > 0.35 or \
                np.abs(x[M.ITILT]).max() > 0.35:
            failed_at = t
            n_steps = k + 1
            break

    for k2 in td_backup:
        P.td[k2] = td_backup[k2]

    out = {k: v[:n_steps] for k, v in logs.items()}
    out["est"] = est_log[:n_steps]
    out["t"] = np.arange(n_steps) * dt
    out["failed_at"] = failed_at
    out["metrics"] = compute_metrics(out, p_cmd)
    return out


def compute_metrics(out, p_cmd):
    t = out["t"]
    met = {}
    met["max_tilt"] = float(out["tilt"][:, 2].max())
    met["max_relPos"] = float(np.abs(out["relPos"]).max()) \
        if len(t) else 0.0
    met["max_obz"] = float(np.abs(out["obz"]).max())
    met["sat_frac"] = float(out["sat"].mean())
    met["failed_at"] = out["failed_at"]
    met["escaped"] = bool(met["max_relPos"] > 0.023)
    # tracking RMS during move windows
    mask = np.linalg.norm(out["cmd_v"], axis=1) > 0.5 * np.linalg.norm(
        p_cmd["profileVelocityWorld"])
    if mask.any():
        err = out["vb"] - out["cmd_v"]
        met["track_rms"] = float(np.sqrt((err[mask] ** 2).mean()))
        met["vb_mean_speed"] = float(np.linalg.norm(
            out["vb"][mask], axis=1).mean())
    else:
        met["track_rms"] = np.nan
        met["vb_mean_speed"] = np.nan
    # wheel-rate noise metric during cruise
    if mask.any():
        met["w_err_rms"] = float(np.sqrt(
            ((out["w_meas"] - out["w_true"]) ** 2)[mask].mean()))
    return met
