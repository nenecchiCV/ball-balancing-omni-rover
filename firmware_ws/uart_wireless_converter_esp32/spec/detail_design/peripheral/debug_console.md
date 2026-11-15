# debug_console

診断・統計のテキスト出力を `usb_serial`（USB-Serial-JTAG）経由で行う薄いラッパ。`debug_console` static class（システムに 1 つ）。

## 公開関数

- `static void write(const char* text, uint32_t len)` — `printk` 経由で文字列を送出する。
- `static void write_line(const char* text)` — 改行付き 1 行送出。

## 内部状態

- なし。

## 備考

- console/shell は Zephyr が `usb_serial` に割り当てる（ボード dts の `zephyr,console`/`zephyr,shell-uart`）。本コンポーネントは UART0 とは別経路として透過ストリームを汚染しない。
