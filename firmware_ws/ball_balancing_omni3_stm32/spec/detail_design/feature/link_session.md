# link_session

ロボットリンクプロトコルのセッション管理（FR-09, FR-11）。受信フレームの振分け・制御モード/指令スロット・イベント購読・安全ウォッチドッグ・イベント生成を担い、リンクスレッドのポンプ本体 `run_once` も提供する。`link_uart`（peripheral）と `link_protocol`（common）にのみ依存する。

## スレッドモデル

- **リンクスレッド**: `robot_application` が生成。`run_once(K_FOREVER)` を繰返し、`rx_sem`/`tx_msgq` の `k_poll` で起床する。受信デコード・応答生成・`link_uart::write`（TX リング投入）はすべてこのコンテキスト。
- **制御スレッド（外側タスク）**: `publish_state`（状態メールボックス更新）・`command`（指令スロット読出）・`generate_events`（EVT 生成→キュー投入）を呼ぶ。いずれも短い整合コピーまたは `k_msgq_put(K_NO_WAIT)` でブロックしない。

## 公開型

```cpp
class link_session {  // static class
 public:
  struct config { const param_set* params; };
  struct robot_state {        // メールボックス: 制御スレッド→リンクスレッド
    bool enable; bool estop; bool imu_ok; bool qualified;
    float raw_x, raw_y, raw_yaw;   // RESET_ODOMETRY / STOP 用の生値
    float x, y, yaw;               // イベント用オフセット済 pose
    float vx, vy, yaw_rate;
    uint8_t ctrl_mode;             // balance_controller 出力 mode
  };
  struct command_output {     // 制御スレッドへの指令スロット読出
    uint8_t mode;             // 0=standby, 1=velocity, 2=position
    vec3 velocity_cmd{};      // mode=velocity 時のワールド速度指令（wdg ゼロ化済み）
  };
  struct stats { uint32_t rx_frames, rx_crc_drops, rx_overruns, tx_drops, last_rx_ms; };
};
```

## 公開関数

- `static void init(const config& cfg)` — `link_uart` 初期化（`DT_ALIAS(link_uart)`）・内部状態リセット。
- `static void run_once()` — `k_poll` で `rx_sem`/`tx_msgq` を待ち、受信排水→フレーム振分け、TX キュー排水を行うポンプ 1 周分（リンクスレッドの無限ループから呼ぶ）。
- `static void publish_state(const robot_state& s)` — メールボックス整合コピー（`irq_lock`）。
- `static command_output command()` — 指令スロット整合コピー。速度指令ウォッチドッグ（最終 `SET_VELOCITY` から `link.watchdog_ms`=500ms）と UART アイドル（`link.idle_ms`=1000ms）をここで評価し、失効時はゼロ指令を返す。アイドル超過時は購読もリセットする。
- `static void generate_events(uint32_t now_ms)` — 購読中の `EVT_POSE` を指定レートで分周送出、`EVT_STATUS` を変化検出 + 10 Hz ハートビートでキュー投入。
- `static void log(uint8_t level, const char* text)` — `EVT_LOG` 生成→キュー投入（どちらのスレッドからも呼べる）。
- `static stats get_stats()` — shell 診断用カウンタ整合コピー。

## 振分け（dispatch）

受信フレームを `link_protocol::request` にデコードして処理する。応答は `link_uart::write` で即送出（リンクスレッド上のため非ブロッキングに TX リングへ入る）。

| TYPE | 処理 | 応答 |
|---|---|---|
| `PING` | — | `PONG(seq, proto_ver=1, caps=0x07)` |
| `SET_VELOCITY` | `mode==VELOCITY` のみ受理して指令スロット更新＋`last_vel_ms` 更新。ペイロード長 12B 必須 | `ACK(result)` — モード不整合は `REJECTED_STATE`、長不整合は `INVALID_PARAM` |
| `SET_POSE_TARGET` | `imu_ok && qualified` 必須。`position_controller::set_target` + `mode=POSITION` へ移行 | `ACK` — 未健全は `NOT_READY` |
| `STOP` | 速度指令スロットゼロ化＋位置モード時は `position_controller::hold_at(current)` | `ACK(OK)` |
| `SET_CONTROL_MODE` | `mode<=2` を受理。`POSITION` は `imu_ok && qualified` 必須。`STANDBY` では指令スロット・目標をクリア | `ACK` |
| `RESET_ODOMETRY` | `odometry::reset_origin(raw_x, raw_y, raw_yaw)`（`irq_lock` で外側タスクと排他） | `ACK(OK)` |
| `SUBSCRIBE` | `event_type ∈ {0x90,0x91,0x92}` かつ `EVT_POSE` の `rate_hz ∈ 1..50`（他は 0 でも可）で購読テーブル更新 | `ACK` |
| `UNSUBSCRIBE` | 購読テーブル削除 | `ACK(OK)` |
| `GET_STATUS` | — | `STATUS(seq, mode, enable, estop, imu_ok, uptime_ms)` |
| その他リクエスト | — | `ACK(result=UNSUPPORTED)` |

`STATUS.mode`・`EVT_STATUS.mode` は `SET_CONTROL_MODE` と同じ定義（リンク側制御モード）。`balance_controller` の mode（0/1/2 倒立状態機械）とは別物で、EVT_STATUS にはリンクモードを載せる。

## イベント生成（generate_events、外側タスク 200 Hz 呼出）

- `EVT_POSE`: `rate_hz` で分周（`outer % (200/rate)`）、`t_ms=now` でエンコードして `tx_msgq` へ。
- `EVT_STATUS`: `(mode,enable,estop,imu_ok)` が前回送出から変化したとき、または 100 ms 経過（10 Hz 上限ハートビート）で送出。
- `EVT_LOG`: 発生時のみ（`log()` からキュー投入）。
- `tx_msgq`（深さ 8、フレーム最大 72B）満杯はドロップして `tx_drops` 計数のみ。

## 内部状態

- `decoder_`（`frame_decoder`）、`rx_scratch[256]`（リング排水用）。
- `mode_`・`vel_cmd_`・`last_vel_ms_`・`last_rx_ms_`（指令スロット、`irq_lock` 保護）。
- `subs_{pose_rate_hz, log, status}`（購読テーブル、`irq_lock` 保護）。
- `state_box_`（`robot_state` メールボックス）。
- `tx_msgq_`（`k_msgq`）、`pose_div_count_`、`prev_status_`、`last_status_tx_ms_`。
- カウンタ `stats_`。

## 備考

- `SET_VELOCITY` の値は `link_session` ではクランプしない。`balance_controller` 側の連成リミット・キャップ経路をそのまま通す（FR-09）。
- 位置制御成立条件の `qualified` は `est.updated_qualification_time >= estimator.bias_qualification_time` を `publish_state` 側で評価して渡す。
- `STOP` 受信時の `hold_at` はメールボックスの現在 pose を使う。
- リンク未確立時は `command()` が `link_active=false` を返し、呼出し側（`robot_application`）は従来の `command_profile` を使う。一度確立したリンクはアイドル失効時に STANDBY・ゼロ指令・購読解除へ遷移し、`command_profile` には戻らない（リンク断後の自律再開を防ぐ安全側の選択）。
- UART RX は ISR→リング→リンクスレッド、イベント TX は制御スレッド→`k_msgq`→リンクスレッドの二段経路で、どちらのスレッドもブロックしない。
