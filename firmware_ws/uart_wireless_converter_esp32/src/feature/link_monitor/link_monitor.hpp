#pragma once

#include <cstdint>

#include "device/link/link.hpp"

namespace uwc {

// リンク状態・転送統計・活動時刻の記録と snapshot 提供。static class。
class link_monitor {
 public:
  link_monitor() = delete;

  struct stats {
    uint32_t bytes_host_to_link;
    uint32_t bytes_link_to_host;
    uint32_t send_fail_count;
    uint32_t drop_bytes;
    uint32_t connect_count;
    uint32_t disconnect_count;
    uint32_t last_activity_ms;
  };

  static void init();
  static void on_event(link::event ev);
  static void note_tx(uint32_t bytes);  // host→link（無線送出）方向
  static void note_rx(uint32_t bytes);  // link→host（UART 送出）方向
  static void note_drop(uint32_t bytes);
  static void note_send_fail();

  static stats snapshot();
  static void reset();
  static bool connected();

 private:
  static inline stats stats_ = {};
  static inline bool connected_ = false;
};

}  // namespace uwc
