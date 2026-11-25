# operation_manager

動作状態（待機 ⇔ 動作中 ⇔ 終端）と IMU キャリブレーション状態を管理する（FR-12, FR-13）。状態遷移・受理判定は Zephyr 非依存の純粋ロジックとし、ホストテスト対象とする。モータ駆動の実体は `robot_application` が `drives_motors()` を参照してゲートする。

## 公開型

```cpp
class operation_manager {  // static class
 public:
  struct config { const param_set* params; };

  // 動作状態（STATUS/EVT_STATUS の op_state と同じ値）
  enum op_state : uint8_t { STANDBY = 0, OPERATING = 1, FAULT = 2, TUNING = 3 };
  // キャリブレーション状態（STATUS/EVT_STATUS の calib_state と同じ値）
  enum calib_state : uint8_t { NONE = 0, RUNNING = 1, DONE = 2, FAILED = 3 };

  // START_OPERATION 受理判定の入力（リンクスレッド側で組立てる）
  struct start_cond {
    bool imu_ok;
    bool estop;
    bool qualified;
    float tilt;  // 推定姿勢の傾斜角 [rad]
  };
};
```

## 公開関数

- `static void init(const config& cfg)` — パラメータ参照を保持し、`STANDBY`・`calib_state::NONE`・未受理シーケンスで初期化する。
- `static uint8_t state()` / `static uint8_t calib()` — `STATUS`/`EVT_STATUS` 用の状態値。`irq_lock` 保護の 1 バイト読出し。
- `static bool drives_motors()` — `state == OPERATING` のとき true。`robot_application` の外側タスクが制御器実行・サーボ出力のゲートに使う。
- `static uint8_t request_start(const start_cond& c)` — `START_OPERATION` 受理判定（リンクスレッドから呼ぶ）。`ack_result` を返し、`OK` のとき `OPERATING` へ遷移する。条件: `state==STANDBY` かつ `c.imu_ok && !c.estop && c.qualified && c.tilt < start_tilt`。`start_tilt`（既定 5 deg）は Simulink の傾斜起動検証値（μ=0.4 現行構成のリカバリ窓 ~5 deg）で、終端転倒角 `fallen_tilt`（35 deg）とは別パラメータ。状態不整合は `REJECTED_STATE`、条件未成立は `NOT_READY`。
- `static uint8_t request_start_shell(const start_cond& c)` — `bb start` 用。`request_start` と同じ判定（受理経路が違うだけ）。
- `static void request_stop()` — `STOP_OPERATION`/`bb stop`。常時受理し、`OPERATING` → `STANDBY`（動作停止）と `FAULT` → `STANDBY`（故障リセット）へ遷移する。再開始可否は `request_start` の受理判定が担う。
- `static uint8_t request_calib(uint8_t seq)` — `CALIBRATE_IMU` 受理判定。`STANDBY` 中のみ `OK`（受理時に `seq` を保持し `calib_state::RUNNING` へ。推定器のバイアスリセット指示は `take_bias_reset_req()` で外へ出す）。`OPERATING`/`FAULT` 中は `REJECTED_STATE`。
- `static int8_t calib_update(uint32_t now_ms, bool qualified, bool imu_ok)` — キャリブレーション進行の完了判定（外側タスクから呼ぶ）。`RUNNING` 中のみ意味を持ち、`qualified` 成立で `0`（成功→`DONE`）、`link.calib_timeout_ms` 超過で `1`（失敗→`FAILED`）、`!imu_ok` で `2`（失敗→`FAILED`）を返す。実施中継続は `-1`。
- `static uint8_t calib_seq()` — 受理時の `seq`（`CALIB_RESULT` エコー用）。
- `static bool take_bias_reset_req()` — `request_calib` 受理で立てたバイアスリセット要求を取得しクリアする。外側タスクがこれを検出して `state_estimator::reset_bias()` を呼ぶ経路とし、リンクスレッド→制御スレッドの跨ぎ呼出しを避ける。
- `static void notify_controller_terminal()` — `OPERATING` 中に制御器が mode 0（終端）に到達したことを外側タスクが通知し、`FAULT` へ遷移する。`STANDBY` 中の呼出しは無視する。
- `static uint8_t request_tune(float dir, float max_duty, float ramp_s, uint8_t seq)` — `TUNE_DEADZONE` 受理判定（リンクスレッドから呼ぶ）。`STANDBY` 中のみ `OK` を返し `TUNING` へ遷移して `tune_deadzone::start` を呼ぶ。`OPERATING`/`FAULT`/`TUNING` 中は `REJECTED_STATE`。
- `static void notify_tune_done()` — 外側タスクが `tune_deadzone::is_done()` を検出して呼び、`TUNING` → `STANDBY` へ戻す。`TUNE_RESULT` の送出は `robot_application` 経由で `link_session` が行う。
- `static bool tuning()` — `state == TUNING` のとき true。外側タスクが `tune_deadzone::step` の駆動と E-Stop 監視に使う。

## スレッドモデル

- **リンクスレッド**: `request_start`/`request_stop`/`request_calib` を `link_session::dispatch` 経由で呼ぶ。状態遷移は `irq_lock` 下の 1 バイト更新のみでブロックしない。
- **制御スレッド（外側タスク）**: `drives_motors` でゲート判定、`calib_update` で進行管理、`notify_controller_terminal` で fault 追従、`take_bias_reset_req` で推定器リセットを取り出す。
- `start_cond` の材料（`imu_ok`/`estop`/`qualified`/`tilt`）は `link_session` の `robot_state` メールボックスに載せ、リンクスレッド側が dispatch 時に組立てる。

## 内部状態

- `state_`（`op_state`）・`calib_`（`calib_state`）・`calib_seq_`・`calib_deadline_ms_` — `irq_lock` 保護。
- `bias_reset_req_` — バイアスリセット要求フラグ。

## 備考

- `FAULT` からは `request_stop()`（`STOP_OPERATION`/`bb stop`）で `STANDBY` へ復帰する（FR-12 の故障リセット）。`START_OPERATION` は `FAULT` 中も `REJECTED_STATE` で、リセット後に受理条件を満たした場合のみ再開始できる。`balance_controller` の `mode0_latch_` は `OPERATING` 中のみ発動し得る構成とし、`STANDBY` 中は制御器 `step` 自体を実行しない（`robot_application` 側のゲート）。
- `OPERATING`/`FAULT` → `STANDBY` 遷移は次の外側タスクから制御器 `step` を止めるだけで即時反映される。トルク遮断にランプは使わない。
- `TUNING` 中は `drives_motors()` は false を返し、外側タスクは `tuning()` を優先して `balance_controller::step` ではなく `tune_deadzone::step` を駆動する（平衡制御は行わず掃引のみ。車体は人が保持する前提）。
- `TUNING` 中の E-Stop は `tune_deadzone::abort` で掃引を中断し `STANDBY` へ戻る（終端へは遷移しない、FR-17）。`STOP_OPERATION` も掃引を中断する。
- `RUNNING` 中の再 `request_calib` は `OK`（再開=バイアス状態の再リセット＋deadline 更新）としてよい。
