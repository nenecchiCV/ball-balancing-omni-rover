# ipc / bindings / types / limits（frontend）

フロントエンドからバックエンドへの唯一の入口。`@tauri-apps/api` を直接 import できるのは `src/lib/` のみとし、command 呼出しは生成物 `bindings.ts` の `commands` を `ipc.ts` が薄くラップする形で隠蔽する。

## bindings.ts（生成物）

`tauri-specta` が debug ビルド起動時に `commands.rs` のシグネチャから生成する。手編集禁止。

```ts
export const commands = {
  bleScan(timeoutMs: number | null): Promise<DiscoveredDevice[]>,
  bleConnect(address: string): Promise<null>,
  robotPing(): Promise<PongInfo>,
  robotSetVelocity(vx, vy, yawRate): Promise<null>,
  watchLinkState(channel: Channel<LinkState>): Promise<null>,
  watchTelemetry(channel: Channel<PoseTelemetry>): Promise<null>,
  watchStatus(channel: Channel<StatusTelemetry>): Promise<null>,
  watchLog(channel: Channel<LogMessage>): Promise<null>,
  // ... 全 command を網羅
};
```

IPC 型は camelCase（`PongInfo.protoVer`、`PoseTelemetry.tMs`/`yawRate`、`RobotStatusInfo.imuOk`/`uptimeMs`）。失敗時は `AppError` tagged union（`{ kind: "ble" | "notConnected" | "timeout" | "badResponse" | "invalidParam" | "rejected" | "send", message? }`）で reject される。

## 公開関数（ipc.ts）

`commands` の既定値ラッパ。シグネチャは bindings と同一で、`timeoutMs` 等の既定値だけ補う。

```ts
bleScan(timeoutMs = 5000)
bleConnect / bleDisconnect / connectionState
robotPing / robotSetVelocity / robotSetPoseTarget / robotStop
robotSetMode / robotResetOdometry / robotGetStatus
robotStartOperation / robotStopOperation / robotSetPose(x, y, yaw)
robotCalibrateImu / robotTestEcho(payload)
robotSubscribe(eventType, rateHz) / robotUnsubscribe(eventType)
```

watch 系ヘルパ（`Channel` を生成して `onmessage` にコールバックを結線する）:

```ts
watchLinkState(cb: (s: LinkState) => void): void
watchTelemetry(cb: (p: PoseTelemetry) => void): void
watchStatus(cb: (s: StatusTelemetry) => void): void
watchLog(cb: (l: LogMessage) => void): void
```

購読解除はコールバック参照を捨てる方式（Channel が GC されると Rust 側 `send` が失敗し転送タスクが終了する）。`app-shell` が起動時に 1 回だけ呼ぶ。

## types.ts

bindings 型の re-export + プロトコル定数。

```ts
export type { DiscoveredDevice_Serialize as DiscoveredDevice, LinkState, LogMessage,
              LogSource, PongInfo, PoseTelemetry, RobotStatusInfo, StatusTelemetry,
              CalibResultInfo, TestEchoInfo } from "./bindings";
export const CONTROL_MODE = { standby: 0, velocity: 1, position: 2 } as const;
export const OP_STATE = { standby: 0, operating: 1, fault: 2 } as const;
export const CALIB_STATE = { none: 0, running: 1, done: 2, failed: 3 } as const;
```

## limits.ts

```ts
export const MAX_SPEED = 0.4;      // m/s
export const MAX_YAW_RATE = 0.8;   // rad/s
export const RC_SEND_HZ = 20;
export const EVT_POSE = 0x90;      // SUBSCRIBE 用の event_type
```

## 備考

- command 名は Rust 側 `#[tauri::command]` 関数名に対応する（bindings 生成で自動一致）。
- `PoseTelemetry` の位置系フィールドは `number | null`（ロボット側未算出）。表示側で `null` を扱うこと。
