# control_tick

TIM6 による 1 kHz 制御ティックを発生させる。`control_tick` static class（システムに 1 つ）。

## 公開関数

- `bool init()` — devicetree の `timers6` ノードからレジスタアドレスとプリスケーラ（1 MHz）を取得し、ARR=999（1 kHz）を設定。`irq_connect_dynamic` で更新割込コールバック `tick_isr_cb` を登録する。カウンタはまだ開始しない。
- `void start()` — 更新割込とカウンタを有効化してティック供給を開始する。
- `void stop()` — 更新割込とカウンタを停止する。
- `void wait()` — 次ティックまで `k_sem_take(K_FOREVER)` でブロックする。
- `struct k_sem* sem()` — ISR 用のセマフォ参照。

## 内部状態

`sem_`（`init()` で `k_sem_init` により初期化）。ISR `tick_isr_cb` は更新フラグをクリアして `k_sem` を give するだけで、処理本体は行わない。static class のため ISR→インスタンスの参照は不要。

## MATLAB との対応

`sampleTime`/`servoSampleTime` の物理周期を供給する。`IRQ_CONNECT` は C++ 関数スコープ内で `.intList` が破棄されるため `irq_connect_dynamic`（`CONFIG_DYNAMIC_INTERRUPTS`）を使用する。
