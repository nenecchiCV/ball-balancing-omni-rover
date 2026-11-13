# task_scheduler

`control_tick` の 1 kHz セマフォを基準に、内側・外側・診断の各タスクを順次実行する。`task_scheduler` static class。

## 公開型

```cpp
struct hooks {
    void (*inner_cb)();
    void (*outer_cb)();
    void (*diag_cb)();
};
```

## 公開関数

- `static void init()` — カウンタ・統計をリセットする。
- `[[noreturn]] static void run(const hooks&)` — 無限ループで `control_tick::wait()` を待ち、5 ティック毎に outer→inner、それ以外は inner のみを呼ぶ。診断は `timebase::millis()` の 200 ms 分周で実行する。セマフォが滞留した場合はオーバーランとして計数し最新 1 ティックのみ実行する（追いつき実行は位相歪みを招くため行わない）。
- `tick_count()` / `overrun_count()` / `max_inner_us()` / `max_outer_us()` — shell の `bb status` 用の実行統計。

## 内部状態

`tick_count_`（内側周期カウント）、`last_diag_ms_`、`overrun_count_`、最大実行時間統計。

## MATLAB との対応・適応事項

外側タスクと内側タスクの実行順序（外側を先に終わらせてコンテキストを更新）は rev2 の ZOH 関係をそのまま反映する。全タスクが同一制御スレッド上で逐次実行されるため、タスク間共有コンテキストは排他される。実行時間は `timebase::cycles()` の差分で計測する。
