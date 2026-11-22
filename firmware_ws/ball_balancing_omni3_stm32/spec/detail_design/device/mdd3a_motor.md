# mdd3a_motor

Cytron MDD3A モータドライバ（MxA/MxB = CW/CCW 2入力、両方とも PWM）。`mdd3a_motor` クラス（3 輪分の複数インスタンスが必要なためインスタンス化可能）。

## 公開型

```cpp
struct config { pwm_output* mxa; pwm_output* mxb; };
```

`mxa` は正転脚（H,L で正転）へ、`mxb` は逆転脚（L,H で逆転）へ配線する。

## 公開関数

- `void init(const config&)` — 設定を保持し `stop()` で初期出力を 0 にする。
- `void set_duty(float duty)` — duty を [-1.0, +1.0] にクランプし、
  MDD3A の CW/CCW 2入力へ以下のように駆動する。
  - `duty >= 0`（正転）: MxA=PWM@duty、MxB=0%
  - `duty < 0`（逆転）: MxA=0%、MxB=PWM@|duty|
  - PWM オフ期間は両入力 L でブレーキとなる。
- `void stop()` — `set_duty(0)`（両入力 0% = 停止）。

## MATLAB との対応

`duty` 指令のモータ出力。MDD3A は DIR 入力を持たず両入力が
PWM 対応のため、符号に応じて駆動脚を切り替える対称な
CW/CCW-PWM 駆動とする。両入力が同時 H にならないこと
（=ブレーキ状態が正転駆動を打ち消さないこと）が正しさの条件。
