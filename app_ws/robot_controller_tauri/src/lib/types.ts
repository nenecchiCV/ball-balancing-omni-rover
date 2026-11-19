// 共有型の入り口。バックエンド往復型は生成物 bindings.ts が正本。
// プロトコル定数（SET_CONTROL_MODE の mode 値等）はここで補完する。

export type {
  AppError,
  DiscoveredDevice_Serialize as DiscoveredDevice,
  LinkState,
  LogMessage,
  LogSource,
  PongInfo,
  PoseTelemetry,
  RobotStatusInfo,
  StatusTelemetry,
} from "./bindings";

export const CONTROL_MODE = { standby: 0, velocity: 1, position: 2 } as const;
export type ControlMode = (typeof CONTROL_MODE)[keyof typeof CONTROL_MODE];
