// UI クランプ・周期定数 — ball_balancing_omni3_stm32 の control_params と同期。

// 速度上限は stores/params.ts の maxSpeed（ロボット GET_PARAMS 由来、
// 既定 0.35 m/s = シム検証値）を使う。
export const MAX_YAW_RATE = 0.8; // rad/s
export const RC_SEND_HZ = 20;
export const POSE_SUBSCRIBE_HZ = 20;

export const SENSORS_SUBSCRIBE_HZ = 20;

// SUBSCRIBE 対象の event_type（robot_link_protocol.md）。
export const EVT_POSE = 0x90;
export const EVT_LOG = 0x91;
export const EVT_STATUS = 0x92;
export const EVT_SENSORS = 0x93;

// エンコーダ生カウント → 車輪角度の換算（firmware control_params.hpp と同期）。
// enc は x4 デコード済みタイマカウントの累積値（起動時 0 起点・符号付き・ラップなし）。
export const ENC_COUNTS_PER_REV = 341.2; // モデルカウント / 出力軸 1 回転
export const ENC_QUADRATURE = 4.0; // 生カウント / モデルカウント
export const ENC_DEG_PER_COUNT = 360 / (ENC_QUADRATURE * ENC_COUNTS_PER_REV);

// LOG レベル表示名。
export const LOG_LEVEL_NAMES = ["DEBUG", "INFO", "WARN", "ERROR"] as const;
