# link_session

ロボットリンクプロトコルのセッション管理（FR-09, FR-11, FR-14）。受信フレームの振分け・制御モード/指令スロット・イベント購読・安全ウォッチドッグ・イベントバッチ生成を担い、リンクスレッドのポンプ本体 `run_once` も提供する。`link_uart`（peripheral）・`link_protocol`（common）・`operation_manager`/`odometry`/`position_controller`（feature）に依存する（いずれもリンクスレッド→feature 方向のみで循環しない）。

## スレッドモデル

- **リンクスレッド**: `robot_application` が生成。`run_once()` を繰返し、`rx_sem`/`evt_sem` の `k_poll` をバッチ期限をタイムアウトとして起床する。受信デコード・単発応答生成・バッチ送出の `link_uart::write`（TX リング投入）はすべてこのコンテキスト。
- **制御スレッド（外側タスク）**: `publish_state`（状態メールボックス更新）・`command`（指令スロット読出）・`generate_events`（EVT エンコード→バッチバッファ投入）・`notify_calib_result`（`CALIB_RESULT` 即時送出）を呼ぶ。バッファ投入は `irq_lock` 下の短いコピーでブロックしない。

## 公開型

```cpp
class link_session {  // static class
 public:
  struct config { const param_set* params; };
  struct robot_state {        // メールボックス: 制御スレッド→リンクスレッド
    bool enable; bool estop; bool imu_ok; bool qualified;
    float tilt;                    // 推定姿勢の傾斜角 [rad]（START 受理判定用）
    float raw_x, raw_y, raw_yaw;   // RESET_ODOMETRY / SET_POSE / STOP 用の生値
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
- `static void run_once()` — `k_poll` をバッチ期限付きタイムアウトで待ち、受信排水→フレーム振分け→バッチフラッシュ判定を行うポンプ 1 周分（リンクスレッドの無限ループから呼ぶ）。
- `static void publish_state(const robot_state& s)` — メールボックス整合コピー（`irq_lock`）。
- `static command_output command()` — 指令スロット整合コピー。速度指令ウォッチドッグ（最終 `SET_VELOCITY` から `link.watchdog_ms`=500ms）と UART アイドル（`link.idle_ms`=1000ms）をここで評価し、失効時はゼロ指令を返す。アイドル超過時は購読もリセットする。
- `static void generate_events(uint32_t now_ms)` — 購読中の `EVT_POSE` を指定レートで分周エンコード、`EVT_STATUS` を変化検出 + 10 Hz ハートビートでエンコードし、**バッチバッファへ投入**する（即時送出しない）。
- `static void log(uint8_t level, const char* text)` — `EVT_LOG` をエンコードしてバッチバッファのログ FIFO へ投入（どちらのスレッドからも呼べる）。
- `static void notify_calib_result(uint8_t result, uint8_t seq, float gyro_bias_z)` — `CALIB_RESULT(seq, result, bias)` をエンコードして `link_uart::write` で**即時送出**する（バッチ経由しない単発応答）。外側タスクが `operation_manager::calib_update` の完了検出時に呼ぶ。
- `static void notify_tune_result(const tune_deadzone::result& r, uint8_t seq)` — `TUNE_RESULT(seq, code, dz_start[3], dz_keep[3], recommended[3])` を**即時送出**する。外側タスクが `tune_deadzone::is_done` 検出時に呼ぶ。
- `static stats get_stats()` — shell 診断用カウンタ整合コピー。

## 振分け（dispatch）

受信フレームを `link_protocol::request` にデコードして処理する。単発応答（`ACK`/`PONG`/`STATUS`/`CALIB_RESULT`/`TEST_ECHO_RESP`）は `link_uart::write` で即送出する（バッチバッファを経由しない）。`op_state`/`calib_state` は `operation_manager` から読む。

| TYPE | 処理 | 応答 |
|---|---|---|
| `PING` | — | `PONG(seq, proto_ver=3, caps=0x17)` |
| `SET_VELOCITY` | `op_state==OPERATING` かつ `mode==VELOCITY` のみ受理して指令スロット更新＋`last_vel_ms` 更新。ペイロード長 12B 必須 | `ACK(result)` — 動作状態/モード不整合は `REJECTED_STATE`、長不整合は `INVALID_PARAM` |
| `SET_POSE_TARGET` | `op_state==OPERATING` かつ `imu_ok && qualified` 必須。`position_controller::set_target` + `mode=POSITION` へ移行 | `ACK` — 待機中は `REJECTED_STATE`、未健全は `NOT_READY` |
| `STOP` | `op_state==OPERATING` のみ受理。速度指令スロットゼロ化＋位置モード時は `position_controller::hold_at(current)` | `ACK` — 待機中は `REJECTED_STATE` |
| `SET_CONTROL_MODE` | `op_state==OPERATING` のみ受理、`mode<=2` を受理。`POSITION` は `imu_ok && qualified` 必須。`STANDBY` では指令スロット・目標をクリア | `ACK` — 待機中は `REJECTED_STATE` |
| `RESET_ODOMETRY` | `odometry::reset_origin(raw_x, raw_y, raw_yaw)`（`irq_lock` で外側タスクと排他） | `ACK(OK)` |
| `START_OPERATION` | `operation_manager::request_start(cond)`。`cond` はメールボックスの `imu_ok/estop/qualified/tilt` から組立て | `ACK(result)` — `request_start` の返値をそのまま返す |
| `STOP_OPERATION` | `operation_manager::request_stop()`＋リンク側 `mode_=STANDBY`・指令スロットゼロ化 | `ACK(OK)` |
| `SET_POSE` | `op_state==STANDBY` のみ受理。`odometry::set_pose(x, y, yaw, raw_x, raw_y, raw_yaw)` | `ACK` — 待機中以外は `REJECTED_STATE`、長不整合は `INVALID_PARAM` |
| `CALIBRATE_IMU` | `operation_manager::request_calib(req.seq)` | `ACK(result)`（受理時の完了結果は別途 `CALIB_RESULT`） |
| `SET_PARAM` | `param_store::set_key(key, value)`。`TUNING` を含む全状態で受理 | `ACK` — 未定義 key/範囲外は `INVALID_PARAM` |
| `SAVE_PARAMS` | `op_state==STANDBY` のみ受理。`param_store::build_blob` → `flash_store::write(slot, len)` 完了後に応答 | `ACK` — 待機中以外は `REJECTED_STATE`、フラッシュ失敗は `NOT_READY` |
| `RESET_PARAMS` | `op_state==STANDBY` のみ受理。`param_store::init_defaults` + `flash_store` 両スロット消去 | `ACK` — 同上 |
| `TUNE_DEADZONE` | `operation_manager::request_tune(req.seq)` 受理時に `tune_deadzone::request_start(dir, max_duty, ramp_s)`（引数省略時は既定値） | `ACK(result)`（完了報告は別途 `TUNE_RESULT`） |
| `GET_PARAMS` | `param_store::key_at` で `start_key` 以降を昇順列挙し 1 フレーム分（最大 30 件）を `PARAMS(seq, total, count, [key, value]*)` で返す | `PARAMS` — `start_key` が範囲外でも空 `PARAMS` を返す |
| `SUBSCRIBE` | `event_type ∈ {0x90,0x91,0x92}` かつ `EVT_POSE` の `rate_hz ∈ 1..50`（他は 0 でも可）で購読テーブル更新 | `ACK` |
| `UNSUBSCRIBE` | 購読テーブル削除 | `ACK(OK)` |
| `GET_STATUS` | — | `STATUS(seq, op_state, mode, enable, estop, imu_ok, calib_state, uptime_ms, ready_mask, tilt)`（16 B。rev 初期は拡張部なしの 11 B） |
| `TEST_ECHO` | ペイロードをそのまま格納 | `TEST_ECHO_RESP(seq, data[N])` |
| その他リクエスト | — | `ACK(result=UNSUPPORTED)` |

`STATUS.mode`・`EVT_STATUS.mode` は `SET_CONTROL_MODE` と同じ定義（リンク側制御モード）。`balance_controller` の mode（0/1/2 倒立状態機械）とは別物で、EVT_STATUS にはリンクモードを載せる。

`STATUS`/`EVT_STATUS` の拡張部は `ready_mask u8` + `tilt f32`（EVT_STATUS は計 11 B、STATUS は計 16 B）。`ready_mask` は `START_OPERATION` 受理条件の充足ビット（`link_protocol::ready_mask`: `0x01`=IMU 正常、`0x02`=E-Stop 解除、`0x04`=qualification 完了、`0x08`=傾斜が `controller.fallen_tilt` 未満）で、`NOT_READY` 拒否時の未充足条件をホスト側で特定できるようにする。`tilt` は推定姿勢の傾斜角 [rad]。`EVT_STATUS` の変化検出は tilt の連続変化で送出が頻発しないよう先頭 7 B（ready_mask まで）のみを比較し、拡張部はハートビート周期で更新される。

## イベントバッチ送出（FR-14）

購読イベントは即時送出せずバッチバッファに集約し、`link.event_batch_ms`（既定 100 ms）周期でフラッシュする。

- **バッチバッファ構成**（`irq_lock` 保護）:
  - `pose_slot_`/`status_slot_`: 各イベントの最新値スロット（エンコード済みフレーム + dirty フラグ）。同周期内の複数発生は最新 1 件に畳み込む。
  - `log_fifo_`: `EVT_LOG` のエンコード済みフレーム FIFO（深さ 4 程度）。満杯時は新規ドロップして `tx_drops` 計数。
- **フラッシュ条件**（リンクスレッドで評価）: `now >= next_flush_ms_` かつバッファ非空、または滞留バイト数が `flush_threshold_`（目安 192 B）以上。フラッシュはスロット・FIFO の内容を 1 回の `link_uart::write` で連結送出し、`next_flush_ms_ = now + event_batch_ms` を更新する。
- **早期起床**: 滞留が `flush_threshold_` に達した時点で `evt_sem_` を give し、リンクスレッドを期限前に起こす。`run_once` の `k_poll` タイムアウトは `next_flush_ms_` までの残り時間。
- 単発応答は従来どおり `dispatch`/`notify_calib_result` から `link_uart::write` へ直行し、バッチを待たない。
- `EVT_POSE` 分周・`EVT_STATUS` 変化検出/ハートビートの判定は従来どおり `generate_events` で行い、生成されたフレームの行き先がバッファに変わるだけとする。

## 内部状態

- `decoder_`（`frame_decoder`）、`rx_scratch[256]`（リング排水用）。
- `mode_`・`vel_cmd_`・`last_vel_ms_`・`last_rx_ms_`（指令スロット、`irq_lock` 保護）。
- `subs_{pose_rate_hz, log, status}`（購読テーブル、`irq_lock` 保護）。
- `state_box_`（`robot_state` メールボックス）。
- `pose_slot_`/`status_slot_`/`log_fifo_`（バッチバッファ）、`next_flush_ms_`、`flush_threshold_`、`evt_sem_`。
- `pose_div_count_`、`prev_status_`、`last_status_tx_ms_`。
- カウンタ `stats_`。

## 備考

- `SET_VELOCITY` の値は `link_session` ではクランプしない。`balance_controller` 側の連成リミット・キャップ経路をそのまま通す（FR-09）。
- 位置制御成立条件の `qualified` は `est.updated_qualification_time >= estimator.bias_qualification_time` を `publish_state` 側で評価して渡す。
- `STOP` 受信時の `hold_at` はメールボックスの現在 pose を使う。
- リンク未確立時は `command()` が `link_active=false` を返し、呼出し側（`robot_application`）は従来の `command_profile` を使う。一度確立したリンクはアイドル失効時に `mode_=STANDBY`・ゼロ指令・購読解除へ遷移し、`command_profile` には戻らない（リンク断後の自律再開を防ぐ安全側の選択）。`op_state` はリンク失効では変更しない（`OPERATING` 中の切断はモータ遮断ではなくその場保持となる。FR-12）。
- UART RX は ISR→リング→リンクスレッド、イベント TX は制御スレッド→バッファ→リンクスレッドの二段経路で、どちらのスレッドもブロックしない。
- バッチバッファの容量は目安として `MAX_FRAME`（70 B）×6 程度（pose 1 + status 1 + log 4）を確保する。
