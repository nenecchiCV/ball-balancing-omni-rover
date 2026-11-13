# debug_uart

診断出力（Zephyr コンソール経由の printk ラッパ）。`debug_uart` static class。UART の運用自体は Zephyr の console/shell バックエンドに委譲する。

## 公開関数

- `static void write(const char*, uint32_t)` — `printk` で 1 文字ずつ送出する。
- `static void write_line(const char*)` — 行出力。

## MATLAB との対応

なし。診断タスクの CSV テレメトリ送出に使う。対話操作は Zephyr shell（`bb status` / `bb telem`）が担う。
