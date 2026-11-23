# tune_deadzone

モータ不感帯の三角掃引同定セッション。`operation_manager` の `TUNING`（`op_state=3`）中に 1 kHz で駆動され、全車輪へ同時に `0 → dir*max_duty → 0` のデューティを印加し、`wheel_speed_meter` の 5 ms 窓輪速で各車輪の立上り・維持限界を同定する。

## 公開型・関数

```cpp
class tune_deadzone {  // static class
 public:
  struct config {
    const param_set* params;   // tune.wake_rate 等を参照
  };
  struct output {
    float duty[3];             // 本 tick で印加するデューティ
    bool active;               // 掃引実行中
  };
  struct measure {
    float onset[3];            // 上り掃引の立上り |duty|（車輪ごと、0=未検出）
    float sustain[3];          // 下り掃引の維持限界 |duty|
    float suggest[3];          // 推奨 deadzone_comp = max(onset, sustain)
  };

  static void init(const config& cfg);
  static bool request_start(float dir, float max_duty, float ramp_s);
  static void request_cancel();                       // E-Stop/STOP_OPERATION 中断
  static output step(const vec3& wheel_rate,
                     bool estop, uint32_t tick_ms);   // 1 kHz 呼出
  static measure result();
  static bool done();            // 掃引完了/中断で確定
  static uint8_t done_result();  // 0=成功, 1=中断, 2=失敗(未回転)
  static void consume_done();    // robot_application が通知後にクリア
};
```

## 動作仕様

- `request_start` は `dir`（±1 符号化）、`max_duty`（0.01–0.8）、`ramp_s`（1–120）をクランプして掃引位相を初期化し、`measure` をクリアする。`TUNE_RESULT` エコー用の `seq` は `operation_manager::tune_seq()` が保持する（本クラスは保持しない）。
- `step` は `t/ramp_s` の三角位相で `duty = dir*max_duty*phase` を `output.duty` に返し、引数の `wheel_rate`（`wheel_speed_meter::rate()`）を監視して車輪ごとに `onset`（上りで `|w|` が閾値超となる最小 |duty|）・`sustain`（下りで `|w|` が閾値未満へ戻る |duty|）を記録する。閾値は `control_params.tune.wake_rate`（既定 2.0 rad/s）。
- 掃引時間 `2*ramp_s` 到達で `done()` を立て `done_result=0` とする。どの車輪も `onset` を検出しなかった場合は `done_result=2`。
- `request_cancel` は `done_result=1` で `done()` を立てる。`step` は done 後 `output.duty` をゼロにして `active=false` を返し続ける。
- `suggest[i]` は `onset[i]`/`sustain[i]` の大きい方、未検出車輪は `0` とする。
- `robot_application` の外側タスクが `operation_manager::tuning()` 中に `step` を駆動し、`done()` で `result()` を `link_session::notify_tune_result`（即時送出）として報告、`operation_manager::request_stop()` で `STANDBY` へ戻る。モータ印加は `mdd3a_motor` へ直接書き込み（`wheel_servo` はバイパス）。

## 備考

- 平衡制御は実行しないため、車体を人が保持して車輪を無負荷にすることが前提（アプリ画面にガイド表示、FR-10）。
- `motor_deadzone_test`（テストアプリ）と同じ掃引原理だが、本番では UART CSV 出力ではなく `TUNE_RESULT` で構造化値を返す。
