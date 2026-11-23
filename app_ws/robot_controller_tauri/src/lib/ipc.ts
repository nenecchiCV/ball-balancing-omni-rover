// Tauri IPC 境界。`@tauri-apps/api` を直接 import できるのはこのモジュールのみ。
// 本体は生成物 `bindings.ts` の `commands` で、ここでは既定値・購読ヘルパを重ねる。

import { Channel } from "@tauri-apps/api/core";
import { commands } from "./bindings";
import type {
  LinkState,
  LogMessage,
  PoseTelemetry,
  StatusTelemetry,
} from "./types";

export const bleScan = (timeoutMs = 5000) => commands.bleScan(timeoutMs);
export const bleConnect = (address: string) => commands.bleConnect(address);
export const bleDisconnect = () => commands.bleDisconnect();
export const connectionState = () => commands.connectionState();

export const robotPing = () => commands.robotPing();
export const robotSetVelocity = (vx: number, vy: number, yawRate: number) =>
  commands.robotSetVelocity(vx, vy, yawRate);
export const robotSetPoseTarget = (x: number, y: number, speedLimit: number) =>
  commands.robotSetPoseTarget(x, y, speedLimit);
export const robotStop = () => commands.robotStop();
export const robotSetMode = (mode: number) => commands.robotSetMode(mode);
export const robotResetOdometry = () => commands.robotResetOdometry();
export const robotGetStatus = () => commands.robotGetStatus();
export const robotStartOperation = () => commands.robotStartOperation();
export const robotStopOperation = () => commands.robotStopOperation();
export const robotSetPose = (x: number, y: number, yaw: number) =>
  commands.robotSetPose(x, y, yaw);
export const robotCalibrateImu = () => commands.robotCalibrateImu();
export const robotTestEcho = (data: number[]) => commands.robotTestEcho(data);
export const robotSubscribe = (eventType: number, rateHz: number) =>
  commands.robotSubscribe(eventType, rateHz);
export const robotUnsubscribe = (eventType: number) =>
  commands.robotUnsubscribe(eventType);

// バックエンド broadcast → Channel 橋渡しの UI 側受け口。
// 返り値の Promise は invoke 完了のみ。購読解除はコールバック参照を捨てる
// （Channel を GC すると Rust 側の send が失敗し受信ループが終了する）。
const watch = <T>(invoke: (ch: Channel<T>) => Promise<null>) => {
  return (cb: (v: T) => void) => {
    const channel = new Channel<T>();
    channel.onmessage = cb;
    void invoke(channel);
  };
};

export const watchLinkState = watch<LinkState>(commands.watchLinkState);
export const watchTelemetry = watch<PoseTelemetry>(commands.watchTelemetry);
export const watchStatus = watch<StatusTelemetry>(commands.watchStatus);
export const watchLog = watch<LogMessage>(commands.watchLog);
