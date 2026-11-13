# status_led

状態表示 LED。`status_led` static class（1 個）。`update()` は診断タスクから定期的に呼ぶ。

## 公開型

```cpp
enum class pattern : uint8_t { OFF, ON, SLOW_BLINK, FAST_BLINK };
struct config { gpio_pin* led; };
```

## 公開関数

- `static void init(const config&)` — LED ピンを保持し消灯する。
- `static void set(pattern)` — 表示パターンを設定する。
- `static void update(uint32_t now_ms)` — パターンに応じて点滅位相を進める（SLOW=500 ms、FAST=125 ms 周期トグル）。

## 内部状態

`led_`、`pattern_`、`phase_`、`last_toggle_ms_`。

## MATLAB との対応

なし。`robot_application` が IMU 検出状態・制御モードに応じて設定する。
