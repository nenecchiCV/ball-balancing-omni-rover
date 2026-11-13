#include "estop_test/estop_test.hpp"

#include <stdio.h>

#include "device/estop_input/estop_input.hpp"
#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/timebase/timebase.hpp"

namespace bb {

namespace {
void print_state(bool ext, bool btn, bool req) {
  char line[96];
  const int n = snprintf(
      line, sizeof(line),
      "estop t=%lu ext_level=%d btn_level=%d requested=%d\r\n",
      static_cast<unsigned long>(timebase::millis()), static_cast<int>(ext),
      static_cast<int>(btn), static_cast<int>(req));
  debug_uart::write(line, static_cast<uint32_t>(n));
}
}  // namespace

void estop_test::init(const config& cfg) {
  external_ = cfg.external;
  button_ = cfg.button;
}

bool estop_test::start_cb(int argc, char** argv) {
  (void)argc;
  (void)argv;
  debug_uart::write_line(
      "estop test: press external E-stop (PC0) or user button (PC13)");
  tick_ = 0;
  prev_req_ = estop_input::requested();
  prev_ext_ = (external_ != nullptr) && external_->read();
  prev_btn_ = (button_ != nullptr) && button_->read();
  print_state(prev_ext_, prev_btn_, prev_req_);
  return true;
}

void estop_test::update_cb() {
  // 10 Hz（period=100 tick）呼出。変化時に即時表示 + 1 秒ごとに
  // ハートビート表示する。
  const bool ext = (external_ != nullptr) && external_->read();
  const bool btn = (button_ != nullptr) && button_->read();
  const bool req = estop_input::requested();
  const bool changed =
      (req != prev_req_) || (ext != prev_ext_) || (btn != prev_btn_);
  ++tick_;
  if (changed || (tick_ % 10U) == 0U) {
    print_state(ext, btn, req);
  }
  prev_req_ = req;
  prev_ext_ = ext;
  prev_btn_ = btn;
}

void estop_test::stop_cb() {
  debug_uart::write_line("estop test: stopped");
}

}  // namespace bb
