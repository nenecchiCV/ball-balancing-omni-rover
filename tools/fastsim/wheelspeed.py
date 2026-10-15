"""Wheel-speed measurement variants.

Baseline: backward difference of rounded encoder counts
(ballbotWheelRateFromDisplacement) -> +-3.68 rad/s flicker per 5 ms.

Observer candidates:
  * ab:      alpha-beta (g-h) filter on the count stream.
  * abff:    alpha-beta + torque feedforward in the predict step
             (w_hat += tau_cmd/J * dt), the model-aided variant.
  * luen:    Luenberger position+rate observer on the wheel dynamics:
             predict w' = (tau_cmd - c*(w - wkin))/J with the same
             contact-compliance model the plant uses, correct on counts.
All observers track per-wheel displacement theta_hat [rad] and rate
w_hat [rad/s]; the correction uses the quantized count residual.
"""
import numpy as np
import params as P
import model as M


class BackwardDifference:
    name = "bdiff"

    def __init__(self):
        self.enc = M.Encoder()

    def rate(self, thw, dt, tau_cmd=None, wkin=None):
        return self.enc.rate(thw, dt)


class AlphaBeta:
    """g-h filter on counts. alpha position, beta rate (per sample)."""

    name = "ab"

    def __init__(self, alpha=0.28, beta=0.03):
        self.a, self.b = alpha, beta
        self.th = None
        self.w = np.zeros(3)

    def rate(self, thw, dt, tau_cmd=None, wkin=None):
        meas = np.round(thw / P.enc_quant) * P.enc_quant
        if self.th is None:
            self.th = meas.copy()
        self.th = self.th + self.w * dt
        r = meas - self.th
        self.th = self.th + self.a * r
        self.w = self.w + (self.b / dt) * r
        return self.w.copy()


class AlphaBetaFF(AlphaBeta):
    """Alpha-beta with motor-model rate prediction."""

    name = "abff"

    def __init__(self, alpha=0.20, beta=0.015):
        super().__init__(alpha, beta)

    def rate(self, thw, dt, tau_cmd=None, wkin=None):
        meas = np.round(thw / P.enc_quant) * P.enc_quant
        if self.th is None:
            self.th = meas.copy()
        if tau_cmd is not None:
            # Predict with the fitted wheel-drive model (diagonal share of
            # the delivered torque, minus the compliance pull to wkin).
            wk = np.zeros(3) if wkin is None else wkin
            acc = 306.0 * (tau_cmd - P.contact_coupling
                           * (self.w - wk))
            self.w = self.w + acc * dt
        self.th = self.th + self.w * dt
        r = meas - self.th
        self.th = self.th + self.a * r
        self.w = self.w + (self.b / dt) * r
        return self.w.copy()


class Luenberger:
    """Model-based observer: predict with wheel+contact dynamics, correct
    on the quantized encoder position with gain L (2x1 per wheel)."""

    name = "luen"

    def __init__(self, l1=6.0, l2=180.0):
        self.L1, self.L2 = l1, l2     # [rad/rad], [rad/s per rad]
        self.th = None
        self.w = np.zeros(3)

    def rate(self, thw, dt, tau_cmd=None, wkin=None):
        meas = np.round(thw / P.enc_quant) * P.enc_quant
        if self.th is None:
            self.th = meas.copy()
        tau = np.zeros(3) if tau_cmd is None else tau_cmd
        wk = np.zeros(3) if wkin is None else wkin
        acc = 306.0 * (tau - P.contact_coupling * (self.w - wk))
        self.w = self.w + acc * dt
        self.th = self.th + self.w * dt
        r = meas - self.th
        self.th = self.th + self.L1 * r
        self.w = self.w + self.L2 * r * dt
        return self.w.copy()


def make(kind, **kw):
    return {"bdiff": BackwardDifference, "ab": AlphaBeta,
            "abff": AlphaBetaFF, "luen": Luenberger}[kind](**kw)
