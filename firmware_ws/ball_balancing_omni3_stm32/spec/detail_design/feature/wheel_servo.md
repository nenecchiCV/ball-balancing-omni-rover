# wheel_servo

`ballbotWheelServoDutyStep` をそのまま移植した内側タスク。`wheel_servo` static class。

## 公開型

```cpp
struct config { const param_set* params; };
struct input {
    vec3 attitude_torque; vec3 wheel_rate_command; bool settle_witness;
    bool enable; vec3 wheel_rate; uint8_t mode;
};
struct output { vec3 duty; vec3 filtered_speed; };
```

## 公開関数

- `static void init(const config&)` — パラメータ参照を保持し、`ramp_clock_` とフィルタ状態をリセットする。
- `static output step(const input&)` — 1 kHz タスクを 1 ステップ実行する。

## 内部状態

- `filtered_speed_`（MATLAB `previousIntegral` 相当、逆起電力補償フィルタ状態）
- `ramp_clock_`（内側ソフトスタートランプの時計）

## MATLAB との対応・適応事項

- 25 ms 時定数のローパス（`back_emf_speed_time_constant`）、推進サーボ（デッドゾーン・P ゲイン・ヘッドルーム・最小シェア・settle 遮断・内側ランプ）、トルク合成・クランプ・電流換算・duty 反転・maxDuty クランプをそのまま実装する。
- `previousIntegral` は外側側の速度積分とは別物で、本クラス内部のフィルタ状態として保持する。
