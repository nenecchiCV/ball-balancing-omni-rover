# pwm_output

単一 PWM チャネルの duty 出力を行う薄いラッパ。`pwm_output` クラス（3 輪分の複数インスタンスが必要なためインスタンス化可能）。キャリア周期・チャネル・極性は devicetree で確定する。

## 公開型

```cpp
struct config { pwm_dt_spec spec; };
```

## 公開関数

- `bool init(const config&)` — `pwm_is_ready_dt` でデバイスを検査する。
- `void set_duty(float duty)` — duty [-1.0, +1.0] をパルス幅へ変換して `pwm_set_pulse_dt`。符号の扱いは呼出側（mdd3a_motor）が担い、本関数は絶対値を出力する。

## 内部状態

`config_`（PWM スペック）。

## MATLAB との対応

なし（ハード依存部）。`mdd3a_motor` の PWM ドライブの基盤。
