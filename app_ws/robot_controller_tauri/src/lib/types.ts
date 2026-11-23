// 共有型の入り口。バックエンド往復型は生成物 bindings.ts が正本。
// プロトコル定数（SET_CONTROL_MODE の mode 値等）はここで補完する。

export type {
  AppError,
  CalibResultInfo,
  DiscoveredDevice_Serialize as DiscoveredDevice,
  LinkState,
  LogMessage,
  LogSource,
  ParamEntry,
  ParamsInfo,
  PongInfo,
  PoseTelemetry,
  RobotStatusInfo,
  StatusTelemetry,
  TestEchoInfo,
  TuneResultInfo,
} from "./bindings";

export const CONTROL_MODE = { standby: 0, velocity: 1, position: 2 } as const;
export type ControlMode = (typeof CONTROL_MODE)[keyof typeof CONTROL_MODE];

// STATUS/EVT_STATUS の op_state 値（robot_link_protocol.md）。
export const OP_STATE = {
  standby: 0,
  operating: 1,
  fault: 2,
  tuning: 3,
} as const;
export type OpState = (typeof OP_STATE)[keyof typeof OP_STATE];

// STATUS/EVT_STATUS の calib_state 値。
export const CALIB_STATE = {
  none: 0,
  running: 1,
  done: 2,
  failed: 3,
} as const;
export type CalibState = (typeof CALIB_STATE)[keyof typeof CALIB_STATE];

// 調整対象パラメータの key ↔ 名称・単位写像（robot_link_protocol.md
// 「調整対象パラメータ」準拠）。ゲイン行列は [row][col] で展開する。
export interface ParamDef {
  key: number;
  name: string;
  unit?: string;
  readOnly?: boolean;
}
export const PARAM_DEFS: ParamDef[] = [
  { key: 0x00, name: "drive.deadzone_comp_pos[0]" },
  { key: 0x01, name: "drive.deadzone_comp_pos[1]" },
  { key: 0x02, name: "drive.deadzone_comp_pos[2]" },
  { key: 0x03, name: "drive.deadzone_comp_neg[0]" },
  { key: 0x04, name: "drive.deadzone_comp_neg[1]" },
  { key: 0x05, name: "drive.deadzone_comp_neg[2]" },
  { key: 0x10, name: "lqi.tracking_gain_scale" },
  { key: 0x11, name: "lqi.anti_windup_gain" },
  { key: 0x12, name: "torque_drive.yaw_damping_gain" },
  { key: 0x13, name: "torque_drive.tilt_integral_gain" },
  { key: 0x20, name: "position.kp" },
  { key: 0x21, name: "position.accel_limit", unit: "m/s²" },
  { key: 0x22, name: "position.arrive_radius", unit: "m" },
  ...Array.from({ length: 30 }, (_, i) => {
    const r = Math.floor(i / 10);
    const c = i % 10;
    return {
      key: 0x30 + r * 10 + c,
      name: `torque_lqi.state_gain[${r}][${c}]`,
    };
  }),
  { key: 0x50, name: "imu.gyro_bias_z", unit: "rad/s" },
  { key: 0x51, name: "imu.ekf_bias_x", unit: "rad/s" },
  { key: 0x52, name: "imu.ekf_bias_y", unit: "rad/s" },
  { key: 0x60, name: "controller.max_speed", unit: "m/s" },
  {
    key: 0x61,
    name: "controller.theoretical_max_speed",
    unit: "m/s",
    readOnly: true,
  },
];
export const paramName = (key: number): string =>
  PARAM_DEFS.find((d) => d.key === key)?.name ?? `0x${key.toString(16)}`;
