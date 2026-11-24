// UI クランプ・周期定数 — ball_balancing_omni3_stm32 の control_params と同期。

// 速度上限は stores/params.ts の maxSpeed（ロボット GET_PARAMS 由来、
// 既定 0.4 m/s = シム検証値）を使う。
export const MAX_YAW_RATE = 0.8; // rad/s
export const RC_SEND_HZ = 20;
export const POSE_SUBSCRIBE_HZ = 20;

export const SENSORS_SUBSCRIBE_HZ = 20;

// SUBSCRIBE 対象の event_type（robot_link_protocol.md）。
export const EVT_POSE = 0x90;
export const EVT_LOG = 0x91;
export const EVT_STATUS = 0x92;
export const EVT_SENSORS = 0x93;

// LOG レベル表示名。
export const LOG_LEVEL_NAMES = ["DEBUG", "INFO", "WARN", "ERROR"] as const;
