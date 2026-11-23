// 共有型の入り口。バックエンド往復型は生成物 bindings.ts が正本。
// プロトコル定数（SET_CONTROL_MODE の mode 値等）はここで補完する。

export type {
  AppError,
  CalibResultInfo,
  DiscoveredDevice_Serialize as DiscoveredDevice,
  LinkState,
  LogMessage,
  LogSource,
  PongInfo,
  PoseTelemetry,
  RobotStatusInfo,
  StatusTelemetry,
  TestEchoInfo,
} from "./bindings";

export const CONTROL_MODE = { standby: 0, velocity: 1, position: 2 } as const;
export type ControlMode = (typeof CONTROL_MODE)[keyof typeof CONTROL_MODE];

// STATUS/EVT_STATUS の op_state 値（robot_link_protocol.md）。
export const OP_STATE = { standby: 0, operating: 1, fault: 2 } as const;
export type OpState = (typeof OP_STATE)[keyof typeof OP_STATE];

// STATUS/EVT_STATUS の calib_state 値。
export const CALIB_STATE = {
  none: 0,
  running: 1,
  done: 2,
  failed: 3,
} as const;
export type CalibState = (typeof CALIB_STATE)[keyof typeof CALIB_STATE];
