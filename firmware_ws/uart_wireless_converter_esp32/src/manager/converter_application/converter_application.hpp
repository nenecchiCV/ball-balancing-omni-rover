#pragma once

#include <zephyr/sys/ring_buffer.h>

#include <cstdint>

#include "feature/link_monitor/link_monitor.hpp"

namespace uwc {

// 全コンポーネントの設定注入・初期化・実行ループを司る composition root。
// static class（システムに 1 つ）。
class converter_application {
 public:
  converter_application() = delete;

  struct config {};

  static bool init(const config& cfg);
  [[noreturn]] static void run();
  static link_monitor::stats stats();
  static void reset_stats();

 private:
  // UART→無線 / 無線→UART の中継リング（静的確保）
  static inline uint8_t h2l_mem_[2048] = {};
  static inline uint8_t l2h_mem_[1024] = {};
  static inline struct ring_buf host_to_link_ = {};
  static inline struct ring_buf link_to_host_ = {};
};

}  // namespace uwc
