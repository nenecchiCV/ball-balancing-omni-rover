#pragma once

#include <cstdint>

namespace uwc {

// 診断出力（usb_serial 上の console 経由の printk ラッパ、static class）。
// データ経路である uart0 とは別経路とし、透過ストリームを汚染しない。
class debug_console {
 public:
  debug_console() = delete;

  static void write(const char* text, uint32_t len);
  static void write_line(const char* text);
};

}  // namespace uwc
