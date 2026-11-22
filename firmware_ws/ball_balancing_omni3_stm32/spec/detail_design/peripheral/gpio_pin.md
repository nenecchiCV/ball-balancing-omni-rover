# gpio_pin

単一 GPIO ピンの入出力と割込設定を行う薄いラッパ。`gpio_pin` クラス（複数インスタンスが必要なためインスタンス化可能なクラスのままとする）。ピンの配置・プル・初期状態は devicetree（`bb,pins` ノードの各 `*-gpios` プロパティ）で確定し、本クラスは `gpio_dt_spec` を保持して Zephyr GPIO API へ委譲する。

## 公開型

```cpp
enum class edge : uint8_t { FALLING, RISING, BOTH };
struct config {
    gpio_dt_spec spec;
    bool output = false;
    bool initial_high = false;
};
```

## 公開関数

- `bool init(const config&)` — `gpio_is_ready_dt` でデバイスを検査し、方向と初期レベルを `gpio_pin_configure_dt` で設定する。
- `bool read() const` — `gpio_pin_get_dt` の結果を返す。
- `void write(bool)` — `gpio_pin_set_dt`。
- `bool init_interrupt(edge)` — `gpio_pin_interrupt_configure_dt` で極性を設定し、`gpio_init_callback` + `gpio_add_callback_dt` で `gpio_irq_cb` を登録する。
- `bool consume_irq_flag()` — ISR で立ったペンディングフラグを取得してクリアする。

## 内部状態

`config_`（ピン情報）、`callback_`（Zephyr コールバック登録構造体）、`irq_pending_`（ISR 発生フラグ、揮発性）。コールバック `gpio_irq_cb` は `CONTAINER_OF` で `callback_` からインスタンスを復元する。

## MATLAB との対応

なし（ハード依存部）。E-Stop・LED の基盤となる。
