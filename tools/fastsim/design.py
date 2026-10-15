"""LQR gain design: port of ballbotRobustTorqueGains + redesign machinery.

The .m file designs a discrete LQR on an 8-state inner plant
[pitch, -roll, q, -p, r, w1..w3] built from the measured B matrix, one per
payload mass on a 0..1 kg grid, then keeps the candidate whose worst-case
closed-loop pole radius across all plants is smallest.  Column scalings
(tilt x1.8, rate x1.0, yaw x5) are applied afterwards.

The redesign adds saturation-aware feasibility constraints and a bandwidth
knob (scale on R), evaluated per candidate on every plant in the family.
"""
import numpy as np
from scipy.linalg import solve_discrete_are, eigvals
import params as P

MEASURED_B0 = np.array([
    [0.0520, -23.5629, 23.5170],
    [-27.1781, 13.5441, 13.6378],
    [22.7010, 22.6692, 22.7372],
    [305.9042, 32.1404, 32.7251],
    [32.7227, 305.9042, 32.1425],
    [32.1404, 32.7227, 305.9652]])

# Reference mass properties at the nominal 0.5 kg payload.  The .m file
# gets these from ballbotPayloadRoverProperties; we reproduce the pivot
# inertia scaling (what matters for B scaling) with our lumped model.
IPIVOT_REF = P.Ipivot
IYAW_REF = P.Iyaw


def payload_props(payload_mass):
    """(mass, com, Ipivot, Iyaw, gravityGain) vs payload [kg]."""
    m_tot, com, I_com = P.payload_rover_props(payload_mass)
    Ipivot = I_com + m_tot * com**2
    Iyaw = P.body_I[2] + payload_mass * P.payload_width**2 / 12
    gg = m_tot * P.G * com / Ipivot
    return m_tot, com, Ipivot, Iyaw, gg


def inner_plant(payload_mass):
    """Continuous (A,B) of the 8-state inner plant at a payload mass."""
    m, com, Ipivot, Iyaw, gg = payload_props(payload_mass)
    B = MEASURED_B0.copy()
    B[0:2, :] *= IPIVOT_REF / Ipivot
    B[2, :] *= IYAW_REF / Iyaw
    A = np.zeros((8, 8))
    A[0, 2] = 1.0
    A[1, 3] = 1.0
    A[2, 0] = gg
    A[3, 1] = gg
    A[5, 5] = A[6, 6] = A[7, 7] = -1.35
    Bp = np.zeros((8, 3))
    Bp[2, :] = B[0, :]
    Bp[3, :] = -B[1, :]
    Bp[4, :] = B[2, :]
    Bp[5:8, :] = B[3:6, :]
    return A, Bp, gg


def c2d_zoh(A, B, Ts):
    """Exact ZOH discretization via matrix exponential."""
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    from scipy.linalg import expm
    eM = expm(M * Ts)
    return eM[:n, :n], eM[:n, n:]


def dlqr(A, B, Q, R):
    S = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ S @ B, B.T @ S @ A)
    return K


PAYLOAD_GRID = np.arange(0.0, 1.0001, 0.25)


def design_state_gain(Q=None, R=None, tilt_scale=1.8, rate_scale=1.0,
                      yaw_scale=5.0, payload_for_hold=0.5,
                      feasible_fn=None):
    """Reproduce ballbotRobustTorqueGains; returns (K10, info).

    feasible_fn(K8, plant_index, gains_meta) -> (bool, detail) can veto a
    candidate gain BEFORE the robust cross-application (saturation-aware
    constraint hook for the redesign).
    """
    if Q is None:
        Q = np.diag([200, 200, 20, 20, 5, 0.2, 0.2, 0.2])
    if R is None:
        R = 4000.0 * np.eye(3)  # mirrors ballbotRobustTorqueGains.m
    plants = []
    gains = []
    for m in PAYLOAD_GRID:
        A, Bp, gg = inner_plant(m)
        Ad, Bd = c2d_zoh(A, Bp, P.Ts)
        plants.append((Ad, Bd, A, Bp, gg))
        gains.append(dlqr(Ad, Bd, Q, R))
    radius = np.zeros((len(plants), len(plants)))
    keep = []
    for i, K in enumerate(gains):
        ok = True
        detail = None
        if feasible_fn is not None:
            ok, detail = feasible_fn(K, i, plants)
        if not ok:
            radius[i, :] = np.inf
        else:
            for j, (Ad, Bd, *_rest) in enumerate(plants):
                radius[i, j] = np.max(np.abs(eigvals(Ad - Bd @ K)))
        keep.append(detail)
    worst = radius.max(axis=1)
    # Tie-break toward the heavy end within a hair of the optimum: the .m
    # notes the winner is always a heavy-end design, and MATLAB's min picks
    # the last strictly-smaller value differently than numpy's argmin on
    # near-ties.
    best = np.min(worst)
    candidates = np.flatnonzero(worst <= best + 1e-4)
    chosen = int(candidates[-1])
    Km = gains[chosen]

    K10 = np.zeros((3, 10))
    K10[:, 2:4] = tilt_scale * Km[:, 0:2]
    K10[:, 4:6] = rate_scale * Km[:, 2:4]
    K10[:, 6] = yaw_scale * Km[:, 4]      # r column (0-indexed 4 = .m col 5)
    # wheel-rate columns 7:10 stay zero (encoder flicker).

    m, com, Ip, Iy, ggNow = payload_props(payload_for_hold)
    tiltRows = MEASURED_B0[0:2, :] * IPIVOT_REF / Ip
    Btilt = np.vstack([tiltRows[0, :], -tiltRows[1, :]])
    leanHold = -np.linalg.pinv(Btilt) @ np.array([ggNow, ggNow])

    info = dict(chosen=chosen, chosenPayload=PAYLOAD_GRID[chosen],
                worstRadius=worst, radius=radius, K8=Km,
                leanHold=leanHold, plants=plants)
    return K10, info


def saturation_metrics(K8, plants=None):
    """Per-wheel torque demand at the lean-setpoint edge.

    Predicted |tau| when the lean demand sits at leanSetpointLimit with the
    tilt deviations zero (pure feedforward hold) is supplied by leanHold;
    the FEEDBACK share at a disturbance equal to the lean band is
    |K_tilt| * leanSetpointLimit per wheel — this must stay under the
    traction clamp or the loop is bang-bang inside the operating cone.
    """
    leanBand = P.td["leanSetpointLimit"]
    tilt_col = np.abs(K8[:, 0:2]).max(axis=1)          # N*m per rad
    tau_fb = tilt_col * leanBand
    return dict(tilt_col=tilt_col, tau_fb_at_lean_edge=tau_fb,
                clamp=P.traction_limited_torque)
