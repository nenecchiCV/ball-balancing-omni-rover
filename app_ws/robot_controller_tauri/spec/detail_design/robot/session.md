# session / telemetry / log（robot ドメイン）

接続ライフサイクル・リクエスト送出・受信振分け・イベント配信を担う中心モジュール。Tauri 非依存であり、`tauri::` を import しない。UI への配信は `tokio::sync::broadcast` のイベントバスで行い、IPC 側（`watch_*` command）が Channel への橋渡しを担う。

`AppState` が `Arc<Session>` を保持し、command 層から呼ばれる。内部同期は `tokio::sync` プリミティブで行う。

## session.rs — 公開型

```rust
pub enum LinkState { Disconnected, Scanning, Connecting, Connected }  // Serialize + Type
pub struct Session { /* NusLink, dispatcher ハンドル, pending マップ, broadcast バス */ }
```

## session.rs — 公開関数

- `Session::new() -> Session`
- `subscribe_link/telemetry/status/log(&self) -> broadcast::Receiver<T>` — イベントバスの受信端。`watch_*` が購読に使う。
- `async fn state(&self) -> LinkState`
- `async fn scan(&self, timeout_ms: u32) -> Result<Vec<DiscoveredDevice>, AppError>` — `ble::nus::scan` に委譲。`LinkState::Scanning` 遷移をバスへ発行する。
- `async fn connect(self: &Arc<Self>, address: &str) -> Result<(), AppError>` — 接続→特性確保→通知購読後、ディスパッチャタスクを spawn する。
- `async fn disconnect(&self)` — ディスパッチャ停止・購読リセット・BLE 切断を行う（失敗しても状態は Disconnected へ戻す）。
- `async fn request(&self, req: &Request) -> Result<Response, AppError>` — SEQ を発行してフレーム送出し、保留マップに `oneshot` を登録して 1 s で待つ。ACK/PONG/STATUS を返す。
- `async fn request_data(&self, req: &Request, expect_type: u8, timeout_ms: u32) -> Result<Response, AppError>` — 2 段応答用。`ACK(result!=0)` は即 `Rejected` で完了、`ACK(OK)` の後に `expect_type` のデータ応答（`CALIB_RESULT`/`TEST_ECHO_RESP`）を `timeout_ms` まで待って返す。`CALIBRATE_IMU` は `calib_timeout_ms`+マージン（目安 6 s）、`TEST_ECHO` は 1 s。
- `async fn send_no_ack(&self, req: &Request) -> Result<(), AppError>` — `SET_VELOCITY` 等高頻度指令用。送出のみで応答を待たない。

## telemetry.rs — 公開型

```rust
#[serde(rename_all = "camelCase")]
pub struct PoseTelemetry { pub t_ms: u32, pub x: Option<f32>, pub y: Option<f32>,
                           pub yaw: Option<f32>, pub vx: Option<f32>,
                           pub vy: Option<f32>, pub yaw_rate: Option<f32> }
pub struct StatusTelemetry { pub op_state: u8, pub mode: u8, pub enable: u8,
                             pub estop: u8, pub imu_ok: u8, pub calib_state: u8 }
```

`Option<f32>` はロボット側が当該値を未算出の場合（`f32::NAN` 相当）を表し、TS では `number | null` となる。

## log.rs — 公開型

```rust
#[serde(rename_all = "camelCase")]
pub enum LogSource { Robot, App }
pub struct LogMessage { pub level: u8, pub text: String, pub source: LogSource }
```

`source` は `EVT_LOG`（robot）とアプリ内イベント（app）を同一リストで扱うための UI 側拡張で、ロボットプロトコルには含めない。

## 内部状態（session）

- `pending_: HashMap<u8, PendingEntry>` — SEQ → 応答チャネル。`PendingEntry` は `OneShot`（最初の応答で完了）か `AckThen`（`ACK(OK)` では待機を継続し、データ応答または NACK で完了）を持つ。応答フレームのヘッダ `SEQ` はロボット側が 0 で送るため、相関には `Response::decode` が取り出すペイロードの seq エコーを使う。`expect_type` の検査は `request_data` 側で行い、期待外の応答は `BadResponse` で完了する。
- `seq_: u8` — リクエストごとにインクリメント（イベント用 `0` と衝突しない範囲は 1–255）。
- `subscriptions_: HashSet<u8>` — 有効な購読 TYPE。
- `bus`: `telemetry`/`status`/`log`/`link` の 4 本の broadcast チャネル。
- ディスパッチャタスク: 通知バイト → `FrameDecoder::push` → 応答/イベント decode → pending 完了またはバス発行（`EVT_LOG` は `LogSource::Robot` を付与）。通知ストリーム終端・`is_connected` 監視で `Disconnected` を発行する。

## バス発行と UI ストリームの対応

| ロボット側 TYPE | バス | ペイロード |
|---|---|---|
| `EVT_POSE` | `telemetry` | `PoseTelemetry` |
| `EVT_STATUS` | `status` | `StatusTelemetry` |
| `EVT_LOG` | `log` | `LogMessage` |
| 接続遷移 | `link` | `LinkState` |

## 備考

- `request` のタイムアウトは 1 s（FR-06）。タイムアウト時は pending エントリを除去して `AppError::Timeout` を返す。
- `send_no_ack` の送信値はここで `limits` 相当のクランプを通す（NFR-02 の 2 段階目）。
- broadcast capacity はテレメトリ高頻度を想定した余裕値とし、`Lagged` 検出時は受信側で最新へ追従する方針とする。
