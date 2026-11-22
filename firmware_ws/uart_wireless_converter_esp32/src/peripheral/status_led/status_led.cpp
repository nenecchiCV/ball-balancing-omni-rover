// ユーザー LED 表示制御（FR-05）。点灯/消灯/1 秒周期点滅を提供し、
// 点滅は system workqueue の遅延ワークで駆動して転送処理をブロックしない。

#include "peripheral/status_led/status_led.hpp"

#include <zephyr/sys/printk.h>

namespace uwc {

namespace {
constexpr uint32_t BLINK_HALF_PERIOD_MS = 500;
}

void status_led::blink_work_cb(struct k_work* work) {
  ARG_UNUSED(work);
  if (mode_ != mode::BLINK) {
    return;
  }
  (void)gpio_pin_toggle_dt(led_);
  (void)k_work_reschedule(&blink_work_, K_MSEC(BLINK_HALF_PERIOD_MS));
}

bool status_led::init(const config& cfg) {
  if (cfg.led == nullptr || !gpio_is_ready_dt(cfg.led)) {
    return false;
  }
  led_ = cfg.led;
  const int err = gpio_pin_configure_dt(led_, GPIO_OUTPUT_INACTIVE);
  if (err != 0) {
    printk("uwc: status_led configure failed (%d)\n", err);
    return false;
  }
  k_work_init_delayable(&blink_work_, &blink_work_cb);
  mode_ = mode::OFF;
  return true;
}

void status_led::set_mode(mode m) {
  if (led_ == nullptr || m == mode_) {
    return;
  }
  mode_ = m;
  switch (m) {
    case mode::OFF:
      (void)k_work_cancel_delayable(&blink_work_);
      (void)gpio_pin_set_dt(led_, 0);
      break;
    case mode::SOLID:
      (void)k_work_cancel_delayable(&blink_work_);
      (void)gpio_pin_set_dt(led_, 1);
      break;
    case mode::BLINK:
      // 点灯から開始し、以後ワークが 500 ms ごとにトグルを繰り返す
      (void)gpio_pin_set_dt(led_, 1);
      (void)k_work_reschedule(&blink_work_, K_MSEC(BLINK_HALF_PERIOD_MS));
      break;
  }
}

}  // namespace uwc
