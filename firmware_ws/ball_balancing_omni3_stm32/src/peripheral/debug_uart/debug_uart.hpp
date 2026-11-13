#pragma once

#include <cstdint>

namespace bb {

// 診断出力（Zephyr コンソール経由の printk ラッパ、static class）。
// UART の運用は Zephyr の console/shell バックエンドに委譲する。
class debug_uart {
 public:
  debug_uart() = delete;

  static void write(const char* text, uint32_t len);
  static void write_line(const char* text);
};

}  // namespace bb
