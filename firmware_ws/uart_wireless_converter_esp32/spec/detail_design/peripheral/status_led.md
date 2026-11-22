# status_led

基板上ユーザー LED（`led0`、GPIO15・アクティブ Low）の表示制御。`status_led` static class（システムに 1 つ）。GPIO ドライバへの依存を peripheral 層に閉じ込める（FR-05）。

## 公開型

```cpp
struct config {
    const struct gpio_dt_spec* led;   /* DT_ALIAS(led0) */
};

enum class mode : uint8_t {
    OFF,     /* 消灯（link::state::IDLE 相当） */
    SOLID,   /* 常時点灯（link::state::CONNECTED 相当） */
    BLINK,   /* 1 秒周期点滅（link::state::WAITING 相当、0.5 s 点灯 / 0.5 s 消灯） */
};
```

## 公開関数

- `static bool init(const config&)` — LED GPIO の準備確認（`gpio_is_ready_dt`）と出力初期化（`GPIO_OUTPUT_INACTIVE`＝消灯）、遅延ワーク `blink_work_` の初期化を行う。失敗時は false。
- `static void set_mode(mode)` — 表示モードを切替える。`OFF`/`SOLID` は点滅ワークをキャンセルして点灯状態を確定し、`BLINK` は点灯状態から開始して 500 ms 後のトグルをスケジュールする。どの実行コンテキスト（BT コールバックスレッド含む）からも呼べる。

## 内部状態

- `led_`（`gpio_dt_spec`）、`mode_`（現在モード）、`blink_work_`（`k_work_delayable`）の static メンバ。
- ワークハンドラ `blink_work_cb` は system workqueue 上で LED をトグルし、モードが `BLINK` の間だけ 500 ms で再スケジュールする。モード変更時のキャンセルとの競合は、ハンドラ先頭で `mode_ == BLINK` を確認して再スケジュールを抑止する方式で吸収する。

## 備考

- 点滅の周期生成は専用スレッドを持たず `k_work_delayable`（system workqueue）で行い、バイトストリーム転送をブロックしない（FR-05）。
- 表示モードの写像（`link::event` → `mode`）は `link_monitor::on_event` が行う。`status_led` 自身は無線リンクを知らない。
