# commands / error / state / app_entry（IPC シェル）

Tauri の IPC 境界。`commands.rs` は `#[tauri::command]` + `#[specta::specta]` 関数群として UI 引数を検証して `robot::Session` へ委譲し、`AppError` で失敗を返す。ドメインロジックは持たない。

`tauri-specta` が command シグネチャから `src/lib/bindings.ts` を生成するため、IPC 往復型は `serde(rename_all = "camelCase")` + `specta::Type` を必須とする。BigInt 相当型（u64 等）は specta-typescript が禁止するため IPC 型では u32 以下とする。

## 公開関数（#[tauri::command]）

```rust
async fn ble_scan(state, timeout_ms: Option<u32>) -> Result<Vec<DiscoveredDevice>, AppError>
async fn ble_connect(state, address: String) -> Result<(), AppError>
async fn ble_disconnect(state) -> Result<(), AppError>
async fn connection_state(state) -> Result<LinkState, AppError>

async fn robot_ping(state) -> Result<PongInfo, AppError>
async fn robot_set_velocity(state, vx: f32, vy: f32, yaw_rate: f32) -> Result<(), AppError>
async fn robot_set_pose_target(state, x: f32, y: f32, speed_limit: f32) -> Result<(), AppError>
async fn robot_stop(state) -> Result<(), AppError>
async fn robot_set_mode(state, mode: u8) -> Result<(), AppError>
async fn robot_reset_odometry(state) -> Result<(), AppError>
async fn robot_start_operation(state) -> Result<(), AppError>
async fn robot_stop_operation(state) -> Result<(), AppError>
async fn robot_set_pose(state, x: f32, y: f32, yaw: f32) -> Result<(), AppError>
async fn robot_calibrate_imu(state) -> Result<CalibResultInfo, AppError>
async fn robot_test_echo(state, payload: Vec<u8>) -> Result<TestEchoInfo, AppError>
async fn robot_get_params(state, start_key: u8) -> Result<ParamsInfo, AppError>
async fn robot_set_param(state, key: u8, value: f32) -> Result<(), AppError>
async fn robot_save_params(state) -> Result<(), AppError>
async fn robot_reset_params(state) -> Result<(), AppError>
async fn robot_tune_deadzone(state, dir: f32, max_duty: f32, ramp_s: f32) -> Result<TuneResultInfo, AppError>
async fn robot_get_status(state) -> Result<RobotStatusInfo, AppError>
async fn robot_subscribe(state, event_type: u8, rate_hz: u8) -> Result<(), AppError>
async fn robot_unsubscribe(state, event_type: u8) -> Result<(), AppError>
```

## 公開関数（watch_* — broadcast → Channel 橋渡し）

UI 向けストリーム。`Channel<T>` 引数を受け、broadcast 受信を `channel.send` へ転送するタスクを spawn する。Channel 破棄で send が失敗しタスク終了。`Lagged` は遅延分を捨てて最新へ追従する。

```rust
async fn watch_link_state(state, channel: Channel<LinkState>) -> Result<(), AppError>
async fn watch_telemetry(state, channel: Channel<PoseTelemetry>) -> Result<(), AppError>
async fn watch_sensors(state, channel: Channel<SensorTelemetry>) -> Result<(), AppError>
async fn watch_status(state, channel: Channel<StatusTelemetry>) -> Result<(), AppError>
async fn watch_log(state, channel: Channel<LogMessage>) -> Result<(), AppError>
```

- `watch_link_state` のみ購読時点の現在値を先に 1 回送信する（初期バッジ表示用）。

## 振る舞い

- 各 command は `Request` を組み立てて `Session` を呼ぶ。`robot_set_velocity` は `send_no_ack` 経路、`robot_calibrate_imu`/`robot_test_echo` は `request_data`（2 段応答）経路、それ以外は `request` で応答を待ち、`ack_to_result` が `result != 0` を `AppError::Rejected` として返す。
- `robot_set_mode` は mode 上限（`control_mode::POSITION`）を検証し、超過は `InvalidParam`。
- `robot_calibrate_imu` は `request_data` で `CALIB_RESULT` を待ち、`CalibResultInfo { result, gyroBiasZ }` を返す。`result!=0`（タイムアウト/IMU異常）も正常応答として返し、失敗表示は UI 側で行う。
- `robot_test_echo` は `request_data` で `TEST_ECHO_RESP` を待ち、往復時間を測って `TestEchoInfo { data, rttMs }` を返す。`data` は送信値と一致する前提のエコーペイロード。
- `robot_get_params` は `PARAMS` を待ち、`ParamsInfo { more, entries: { key, value }[] }` を返す。全件取得は UI 側で `start_key` 巡回する。
- `robot_tune_deadzone` は `request_data` で `TUNE_RESULT` を待つ。タイムアウトは `2*ramp_s + 5 s` 程度とし、`TuneResultInfo { result, dzStart[3], dzKeep[3], recommended[3] }` を返す。
- `PongInfo` は `{ protoVer, robotCaps }`、`RobotStatusInfo` は `{ opState, mode, enable, estop, imuOk, calibState, uptimeMs }`（camelCase）。

## error.rs（AppError）

```rust
#[serde(tag = "kind", content = "message", rename_all = "camelCase")]
enum AppError {
    Ble(String),        // btleplug::Error から From 変換
    NotConnected,
    Timeout,
    BadResponse,
    InvalidParam(String),
    Rejected(String),   // ACK result != 0
    Send(String),
}
```

TS 側では `{ kind: "ble", message } | { kind: "notConnected" } | ...` の tagged union として現れる。

## state.rs（AppState）

- `session: Arc<Session>` — 唯一のリンク管理インスタンス。`tauri::Builder::manage` で共有する。内部同期は `Session` 側が担い、IPC シェルは `Mutex` を持たない。

## lib.rs（app_entry）

- `specta_builder()`: `tauri_specta::Builder` に `collect_commands!` で全 command を登録し、`invoke_handler()` を Tauri Builder へ渡す。
- debug ビルド起動時（`cfg(all(debug_assertions, desktop))`）に `../src/lib/bindings.ts` をエクスポートする。モバイルでは相対パスが解決できず起動直後に abort するためデスクトップ限定とする。
- `Builder::default()` に `AppState::new()`・invoke handler を登録して `run()`。
- `tauri_plugin_shell` はデスクトップ専用のため `#[cfg(desktop)]` でのみ登録する（依存も `cfg(not(any(target_os = "android", target_os = "ios")))` 限定）。モバイルでは `shell:default` 権限を持たない（`capabilities/desktop.json` が `"platforms"` でデスクトップに限定）。

## 備考

- command は `Session` の内部同期（ロックは送出区間のみ）に委ね、command 側で長時間ロックを保持しない。
- `watch_*` を含む IPC シェルは GUI ランタイム（wry）を引き込むため `cargo test` の対象外とし、単体テストは Tauri 非依存モジュールに限定する（Windows ではマニフェスト非埋込のテストバイナリが comctl32 v6 未解決でロード失敗するため）。
