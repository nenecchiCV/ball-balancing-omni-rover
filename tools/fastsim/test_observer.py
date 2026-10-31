"""Standalone component test of the PR#6 wheel-rate observer.

Synthesizes a wheel-rate ground truth from the same wheel-drive model the
observer predicts with (w' = driveGain*(tau - compliance*(w - wkin))),
quantizes position to encoder ticks, and compares rate estimates:

  bdiff    : backward difference of rounded counts (raw measurement input)
  lag      : legacy path, first-order lag tc=0.03 s (pre-PR#6 default)
  obs      : PR#6 observer as merged (drag outside the driveGain product)
  obs_fix  : PR#6 observer with drag inside driveGain (plant-faithful)

Profile: rest -> torque ramp -> cruise at 0.306 m/s wheel kinematics ->
stop -> rest.  Reports RMS error, rest-noise, transient tracking, and
robustness under model-parameter mismatch (driveGain perturbed).
"""
import numpy as np
import params as P

DT = P.Ts                     # 0.005 s
Q = P.enc_quant               # 2*pi/341.2
DG, CP = 306.0, 0.134         # nominal plant constants


def profile(t):
    """Torque command (3,) and kinematic rate (3,) at time t."""
    tau = np.zeros(3)
    wkin = np.zeros(3)
    # +x translate: wheels 1,2 at +-10 rad/s cruise (wkin), w0 ~0
    if 2.0 <= t < 4.0:
        tau = np.array([0.0, 0.05, -0.05])
        s = (t - 2.0) / 2.0
        wkin = np.array([0.0, 10.0 * s, -10.0 * s])
    elif 4.0 <= t < 8.0:
        wkin = np.array([0.0, 10.0, -10.0])
        tau = np.array([0.0, 0.01, -0.01])
    elif 8.0 <= t < 10.0:
        tau = np.array([0.0, -0.04, 0.04])
        s = max(0.0, (10.0 - t) / 2.0)
        wkin = np.array([0.0, 10.0 * s, -10.0 * s])
    return tau, wkin


def run(dg_obs=DG, cp_obs=CP, dg_plant=DG, cp_plant=CP):
    T = np.arange(0.0, 12.0, DT)
    n = len(T)
    w_true = np.zeros(3)
    thw = np.zeros(3)
    est = {k: np.zeros(3) for k in ("bdiff", "lag", "obs", "obs_fix")}
    prev_count = np.zeros(3)
    prev_tau = np.zeros(3)
    prev_kin = np.zeros(3)
    out = {k: np.zeros((n, 3)) for k in est}
    out["true"] = np.zeros((n, 3))
    out["meas"] = np.zeros((n, 3))
    for k, t in enumerate(T):
        tau, wkin = profile(t)
        w_true += dg_plant * (tau - cp_plant * (w_true - wkin)) * DT
        thw += w_true * DT
        count = np.round(thw / Q)
        meas = (count - prev_count) * Q / DT
        prev_count = count.copy()
        est["bdiff"] = meas.copy()
        est["lag"] += (DT / (0.03 + DT)) * (meas - est["lag"])
        # merged form: driveGain*tau - compliance*(w - wkin)
        est["obs"] += (dg_obs * prev_tau
                       - cp_obs * (est["obs"] - prev_kin)) * DT
        est["obs"] += 0.15 * (meas - est["obs"])
        # plant-faithful form: driveGain*(tau - compliance*(w - wkin))
        est["obs_fix"] += (dg_obs * (prev_tau
                           - cp_obs * (est["obs_fix"] - prev_kin))) * DT
        est["obs_fix"] += 0.15 * (meas - est["obs_fix"])
        prev_tau = tau
        prev_kin = wkin
        for kname in est:
            out[kname][k] = est[kname]
        out["true"][k] = w_true
        out["meas"][k] = meas
    return T, out


def report(tag, T, out):
    true = out["true"]
    rest = (T < 2.0) | (T >= 10.0)
    cruise = (T >= 4.0) & (T < 8.0)
    tr = np.zeros(len(T), bool)
    for ts in (2.0, 4.0, 8.0, 10.0):
        tr |= (T >= ts) & (T < ts + 0.4)
    print(f"--- {tag} ---")
    for name in ("bdiff", "lag", "obs", "obs_fix"):
        e = out[name] - true
        print(f"{name:8s} rms={np.sqrt((e**2).mean()):7.4f}  "
              f"rest_std={e[rest].std():7.4f}  "
              f"cruise_rms={np.sqrt((e[cruise]**2).mean()):7.4f}  "
              f"transient_rms={np.sqrt((e[tr]**2).mean()):7.4f}  "
              f"steady_rms={np.sqrt((e[~tr]**2).mean()):7.4f}")


if __name__ == "__main__":
    T, out = run()
    report("nominal", T, out)
    for dg, tag in [(DG * 0.8, "driveGain -20%"),
                    (DG * 1.2, "driveGain +20%")]:
        T, out = run(dg_obs=dg)
        report(tag, T, out)
