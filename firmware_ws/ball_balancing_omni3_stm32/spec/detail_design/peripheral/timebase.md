# timebase

時刻・サイクル計測（Zephyr kernel API の薄いラッパ）。`timebase` static class。

## 公開関数

- `static uint32_t millis()` — `k_uptime_get_32`。
- `static uint32_t micros()` — `k_cyc_to_us_floor32(k_cycle_get_32())`。
- `static uint32_t cycles()` — `k_cycle_get_32`。
- `static uint32_t cycles_to_us(uint32_t)` — `k_cyc_to_us_floor32` でサイクル差分を µs へ換算。

## MATLAB との対応

なし。実行時間計測（`max_inner_us`/`max_outer_us`）と周期計時に使う。
