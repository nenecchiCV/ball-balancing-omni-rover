#include "device/estop_input/estop_input.hpp"

namespace bb {

bool estop_input::init(const config& cfg) {
  external_ = cfg.external;
  button_ = cfg.button;
  bool ok = true;
  if (external_ != nullptr) {
    ok = external_->init_interrupt(gpio_pin::edge::BOTH) && ok;
  }
  if (button_ != nullptr) {
    ok = button_->init_interrupt(gpio_pin::edge::FALLING) && ok;
  }
  return ok;
}

bool estop_input::requested() {
  bool stop = false;
  if (external_ != nullptr) {
    // active_low 設定済み: read() が true で「停止要求あり」。
    stop = stop || external_->read() || external_->consume_irq_flag();
  }
  if (button_ != nullptr) {
    stop = stop || button_->consume_irq_flag();
  }
  return stop;
}

}  // namespace bb
