"""Port of the torque-LQI path of ballbotLqiControllerUpdate.m.

Persistent MATLAB states become object attributes. `state_gain`/`int_gain`
come from design.py (ballbotRobustTorqueGains equivalent or a redesign).
"""
import numpy as np
import params as P


def _startup_ramp(x, enable, mode, ramp_state):
    """startupTorqueRampTime soft-start on disturbance paths only."""
    t = P.td["startupTorqueRampTime"]
    if t <= 0 or mode != 1:
        ramp_state[0] = 0.0
        return x
    if not enable:
        ramp_state[0] = 0.0
        return x
    ramp_state[0] = min(ramp_state[0] + P.Ts / t, 1.0)
    return ramp_state[0] * x


class TorqueLqiController:
    def __init__(self, state_gain, integral_gain=None,
                 lean_hold_torque=None):
        self.K = state_gain                        # 3x10
        self.Ki = (np.zeros((3, 2)) if integral_gain is None
                   else integral_gain)
        self.leanHold = (np.zeros(3) if lean_hold_torque is None
                         else lean_hold_torque)
        self.prev_integral = np.zeros(2)
        self.prev_yaw_bias_ready = False
        # persistents
        self.body_rate_filter = None
        self.wheel_rate_filter = None
        self.prev_command_velocity = None
        self.accel_demand_slew = None
        self.tilt_integral = None
        self.prev_torque_command = None
        self.wheel_rate_obs = None
        self.obs_prev_torque = np.zeros(3)
        self.obs_prev_kin = np.zeros(3)
        self._ramp = [0.0]

    def step(self, estimate, command, enable, wheel_rate):
        c = P.ctrl
        td = P.td
        dt = P.Ts
        roll, pitch, yaw = estimate[0], estimate[1], estimate[2]
        bodyRate = estimate[3:6]
        velocityWorld = estimate[6:8]
        contactConf = estimate[13]
        tiltMag = np.hypot(roll, pitch)

        yawBiasReady = self.prev_yaw_bias_ready or (
            abs(bodyRate[2]) < c["yawBiasReadyRateThreshold"])
        self.prev_yaw_bias_ready = yawBiasReady

        if not enable or tiltMag >= c["fallenTilt"]:
            mode = 0
        elif tiltMag >= c["recoveryTilt"] or \
                contactConf < c["minimumContactConfidence"]:
            mode = 2
        else:
            mode = 1

        RBW = np.array([[np.cos(yaw), np.sin(yaw)],
                        [-np.sin(yaw), np.cos(yaw)]])
        velocityBody = RBW @ velocityWorld
        commandVelocityBody = RBW @ command[0:2]
        commandSpeed = np.linalg.norm(commandVelocityBody)
        if commandSpeed > c["maxSpeed"]:
            commandVelocityBody *= c["maxSpeed"] / commandSpeed
            commandSpeed = c["maxSpeed"]

        # follow-through braking
        if td["stopFollowGain"] > 0:
            ballVelW0 = np.array([P.Rb * estimate[9],
                                  -P.Rb * estimate[8]])
            if commandSpeed < td["stopFollowGain"] * np.linalg.norm(ballVelW0):
                commandVelocityBody = RBW @ (td["stopFollowGain"] * ballVelW0)
                commandSpeed = np.linalg.norm(commandVelocityBody)

        if mode == 1:
            velErr = commandVelocityBody - RBW @ np.array(
                [P.Rb * estimate[9], -P.Rb * estimate[8]])
            candInt = self.prev_integral + dt * velErr
            candInt = np.clip(candInt, -c["velocityIntegralLimit"],
                              c["velocityIntegralLimit"])
        else:
            candInt = np.zeros(2)

        state = np.concatenate([
            velocityBody, [pitch, -roll], [bodyRate[1], -bodyRate[0]],
            [bodyRate[2]], np.asarray(wheel_rate, float).ravel()])
        yawRateCmd = min(max(command[2], -c["maxYawRate"]), c["maxYawRate"])
        if not yawBiasReady and abs(command[2]) <= \
                c["yawBiasCommandBypassThreshold"]:
            yawRateCmd = 0.0

        # --- torque-LQI path ---
        if mode != 1:
            self.body_rate_filter = bodyRate.copy()
        else:
            tau = td["rateFilterTimeConstant"]
            if self.body_rate_filter is None:
                self.body_rate_filter = bodyRate.copy()
            elif tau > 0:
                w = dt / (tau + dt)
                self.body_rate_filter += w * (bodyRate
                                              - self.body_rate_filter)
            else:
                self.body_rate_filter = bodyRate.copy()
        filteredBodyRate = self.body_rate_filter
        state[4:7] = [filteredBodyRate[1], -filteredBodyRate[0],
                      filteredBodyRate[2]]

        obs_on = (td.get("wheelRateObserverEnabled", False)
                  and td.get("wheelRateObserverGain", 0) > 0)
        if obs_on:
            w_meas = np.asarray(wheel_rate, float).ravel()
            if mode != 1 or self.wheel_rate_obs is None:
                self.wheel_rate_obs = w_meas.copy()
                self.obs_prev_torque = np.zeros(3)
                self.obs_prev_kin = w_meas.copy()
            else:
                self.wheel_rate_obs += (
                    td["wheelRateObserverDriveGain"] * self.obs_prev_torque
                    - td["wheelRateObserverCompliance"]
                    * (self.wheel_rate_obs - self.obs_prev_kin)) * dt
                self.wheel_rate_obs += td["wheelRateObserverGain"] * (
                    w_meas - self.wheel_rate_obs)
            filteredWheelRate = self.wheel_rate_obs
        else:
            if mode != 1:
                self.wheel_rate_filter = np.asarray(wheel_rate,
                                                    float).ravel()
            else:
                tau = td["wheelRateFilterTimeConstant"]
                if self.wheel_rate_filter is None:
                    self.wheel_rate_filter = np.asarray(wheel_rate,
                                                        float).ravel()
                elif tau > 0:
                    w = dt / (tau + dt)
                    self.wheel_rate_filter += w * (
                        np.asarray(wheel_rate, float).ravel()
                        - self.wheel_rate_filter)
            filteredWheelRate = self.wheel_rate_filter

        yg = td["yawGate"]
        stationary = (np.linalg.norm(velocityBody) < yg["maxSpeed"]
                      and tiltMag < yg["maxTilt"] and commandSpeed == 0)
        yawActive = stationary and abs(command[2]) > yg["commandThreshold"]
        state[6] = state[6] - yawRateCmd if yawActive else 0.0

        if tiltMag > c["tiltPriorityStart"]:
            candInt = np.zeros(2)
        candInt = candInt * (1 - td["integralLeak"] * dt)
        if not td["integralEnabled"]:
            candInt = np.zeros(2)
        if not td.get("wheelRateFeedbackEnabled", True):
            state[7:10] = 0.0

        # outer loop channel 1: velocity error -> lean setpoint
        velLean = td["velocityLeanGain"]
        accelDemand = np.zeros(2)
        if velLean != 0:
            ballVelW = np.array([P.Rb * estimate[9], -P.Rb * estimate[8]])
            ballVelB = RBW @ ballVelW
            velErrB = commandVelocityBody - ballVelB
            accelDemand = velLean * velErrB
            if td["accelFeedforwardGain"] != 0:
                if self.prev_command_velocity is None or mode != 1:
                    self.prev_command_velocity = commandVelocityBody.copy()
                cmdAccel = (commandVelocityBody
                            - self.prev_command_velocity) / dt
                self.prev_command_velocity = commandVelocityBody.copy()
                accelDemand += td["accelFeedforwardGain"] * cmdAccel
            if "positionLeanGain" in td:
                relVelB = RBW @ (estimate[6:8] - ballVelW)
                accelDemand = (accelDemand
                               - td["positionLeanGain"] * estimate[11:13]
                               - td["positionLeanDampingGain"] * relVelB)
            leanLimit = td["leanSetpointLimit"]
            accelDemand = np.clip(accelDemand, -leanLimit * P.G,
                                  leanLimit * P.G)
            if td["leanRateLimit"] > 0:
                if self.accel_demand_slew is None or mode != 1:
                    self.accel_demand_slew = accelDemand.copy()
                else:
                    step = td["leanRateLimit"] * P.G * dt
                    d = np.clip(accelDemand - self.accel_demand_slew,
                                -step, step)
                    self.accel_demand_slew += d
                accelDemand = self.accel_demand_slew
            if "leanFadeTilt" in td:
                excess = max(tiltMag - leanLimit, 0.0)
                fade = max(1.0 - excess / td["leanFadeTilt"], 0.0)
                accelDemand = fade * accelDemand
            state[2:4] -= np.array([accelDemand[0],
                                    -accelDemand[1]]) / P.G

        if "tiltStiffenGain" in td:
            stiffenStart = td["leanSetpointLimit"]
            band = td["tiltStiffenTilt"]
            boost = 1 + (td["tiltStiffenGain"] - 1) * min(
                max(tiltMag - stiffenStart, 0.0) / band, 1.0)
            state[2:6] *= boost

        raw = -self.K @ state - self.Ki @ candInt

        if td["tiltIntegralGain"] != 0:
            if self.tilt_integral is None or mode != 1:
                self.tilt_integral = np.zeros(2)
            else:
                self.tilt_integral += dt * state[2:4]
                self.tilt_integral *= (1 - td["tiltIntegralLeak"] * dt)
                self.tilt_integral = np.clip(
                    self.tilt_integral, -td["tiltIntegralLimit"],
                    td["tiltIntegralLimit"])
            raw = raw - td["tiltIntegralGain"] * (self.K[:, 2:4]
                                                  @ self.tilt_integral)

        if "leanHoldTorque" in td and velLean != 0:
            hold = self.leanHold * np.array([accelDemand[0],
                                             -accelDemand[1]]) / P.G
            raw = raw + _startup_ramp(hold, enable, mode, self._ramp)

        if not yawActive:
            raw = raw - raw.mean()

        kinematicRate = P.W_map @ np.array([velocityBody[0],
                                            velocityBody[1], bodyRate[2]])

        if td.get("positionGain", 0) != 0:
            ballVelW = np.array([P.Rb * estimate[9], -P.Rb * estimate[8]])
            relVelB = RBW @ (estimate[6:8] - ballVelW)
            raw = raw + P.W_map[:, 0:2] @ (
                td["positionGain"] * estimate[11:13]
                + td["positionDampingGain"] * relVelB)

        if td["ballYawGain"] != 0:
            bt = td["ballYawGain"] * estimate[10]
            bt = min(max(bt, -td["ballYawTorqueLimit"]),
                     td["ballYawTorqueLimit"])
            raw = raw + bt * np.ones(3)

        if (td["yawDampingGain"] != 0 and not yawActive
                and commandSpeed > 0):
            raw = raw - td["yawDampingGain"] * filteredBodyRate[2] \
                * np.ones(3)

        slipRate = filteredWheelRate - kinematicRate
        dz = td.get("filteredSlipDeadzone", td["slipDeadzone"])
        slipActive = np.sign(slipRate) * np.maximum(
            np.abs(slipRate) - dz, 0.0)
        raw = raw - td["slipDampingGain"] * slipActive

        if td["propulsionGain"] != 0:
            wrc = P.W_map @ np.array([commandVelocityBody[0],
                                      commandVelocityBody[1], 0.0])
            if "propulsionRelPosGain" in td:
                ballVelW = np.array([P.Rb * estimate[9],
                                     -P.Rb * estimate[8]])
                relVelB = RBW @ (estimate[6:8] - ballVelW)
                relCorr = (td["propulsionRelPosGain"] * estimate[11:13]
                           + td["propulsionRelVelGain"] * relVelB)
                wrc = wrc + P.W_map[:, 0:2] @ relCorr
            rateErr = wrc - filteredWheelRate
            rateActive = np.sign(rateErr) * np.maximum(
                np.abs(rateErr) - dz, 0.0)
            torqueLimit = min(P.continuous_torque_limit,
                              P.traction_limited_torque)
            prop = np.clip(td["propulsionGain"] * rateActive,
                           -td["propulsionTorqueLimit"],
                           td["propulsionTorqueLimit"])
            headroom = np.maximum(torqueLimit - np.abs(raw), 0.0)
            headroom = np.maximum(headroom,
                                  td["propulsionMinShare"] * torqueLimit)
            prop = np.sign(prop) * np.minimum(np.abs(prop), headroom)
            prop = _startup_ramp(prop, enable, mode, self._ramp)
            raw = raw + prop

        limit = min(P.continuous_torque_limit,
                    P.traction_limited_torque) * np.ones(3)
        wheelTorque = np.clip(raw, -limit, limit)

        if td["torqueSlewLimit"] > 0 and mode == 1:
            if self.prev_torque_command is None or not enable:
                self.prev_torque_command = np.zeros(3)
            step = td["torqueSlewLimit"] * dt
            wheelTorque = self.prev_torque_command + np.clip(
                wheelTorque - self.prev_torque_command, -step, step)
            self.prev_torque_command = wheelTorque

        if mode == 2:
            raw2 = -c["recoveryGainScale"] * self.K @ state \
                - td["slipDampingGain"] * slipActive
            wheelTorque = np.clip(raw2, -limit, limit)
        elif mode == 0:
            wheelTorque = np.zeros(3)

        if mode == 1 and td.get("wheelRateObserverEnabled", False):
            self.obs_prev_torque = wheelTorque.copy()
            self.obs_prev_kin = kinematicRate.copy()
        if mode == 1:
            mEff = P.rover_mass + P.Mb + P.Ib_scalar / P.Rb**2
            accRes = P.W_map[:, 0:2].T @ (wheelTorque - raw) / mEff
            candInt = candInt + dt * P.lqi_antiwindup * accRes
            self.prev_integral = np.clip(candInt,
                                         -c["velocityIntegralLimit"],
                                         c["velocityIntegralLimit"])
        else:
            self.prev_integral = np.zeros(2)

        return wheelTorque, mode, yawBiasReady
