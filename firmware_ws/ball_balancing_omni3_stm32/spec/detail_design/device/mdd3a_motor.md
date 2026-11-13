# mdd3a_motor

Cytron MDD3A モータドライバ（PWM + DIR）。`mdd3a_motor` クラス（3 輪分の複数インスタンスが必要なためインスタンス化可能）。

## 公開型

```cpp
struct config { pwm_output* pwm; gpio_pin* dir; };
```

## 公開関数

- `void init(const config&)` — 設定を保持し `stop()` で初期出力を 0 にする。
- `void set_duty(float duty)` — duty を [-1.0, +1.0] にクランプ。DIR=H で正転（`duty>=0` で H を出力）、PWM には絶対値を与える。
- `void stop()` — `set_duty(0)`。

## MATLAB との対応

`duty` 指令のモータ出力。MDD3A の mx_a=PWM / mx_b=DIR 配線に合わせて符号→DIR、絶対値→PWM へ分離する。
