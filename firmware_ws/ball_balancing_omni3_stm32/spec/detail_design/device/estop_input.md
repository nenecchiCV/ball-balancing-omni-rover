# estop_input

非常停止入力（外部 E-stop + 基板ユーザボタン）。`estop_input` static class（1 系統）。

## 公開型

```cpp
struct config {
    gpio_pin* external;  // 外部 E-stop（アクティブロー）。nullptr 可。
    gpio_pin* button;    // 基板ユーザボタン。nullptr 可。
};
```

## 公開関数

- `static bool init(const config&)` — 入力ピンを保持し、外部入力は `edge::BOTH`、ボタンは `edge::FALLING` の EXTI 割込を `gpio_pin::init_interrupt` で有効化する。
- `static bool requested()` — 外部入力は「現在レベル OR 割込発生」、ボタンは「割込発生」で停止要求を判定する。

## 内部状態

`external_` / `button_`（入力ピン）。

## MATLAB との対応

なし（安全入力）。`requested()=true` は外側タスクの `enable=false` へ反映される。
