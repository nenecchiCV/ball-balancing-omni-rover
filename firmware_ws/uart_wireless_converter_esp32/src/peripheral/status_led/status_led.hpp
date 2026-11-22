#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <cstdint>

namespace uwc {

// 基板上ユーザー LED (led0) の表示制御。static class。
// GPIO ドライバへの依存を peripheral 層に閉じ込める。
class status_led {
 public:
  status_led() = delete;

  struct config {
    const struct gpio_dt_spec* led;  // DT_ALIAS(led0)
  };

  // 表示モード（link::state の IDLE/WAITING/CONNECTED に対応）
  enum class mode : uint8_t {
    OFF,    // 消灯
    SOLID,  // 常時点灯
    BLINK,  // 1 秒周期点滅（0.5 s 点灯 / 0.5 s 消灯）
  };

  static bool init(const config& cfg);

  // 表示モード切替。どの実行コンテキストからも呼べる。
  static void set_mode(mode m);

 private:
  static void blink_work_cb(struct k_work* work);

  static inline const struct gpio_dt_spec* led_ = nullptr;
  static inline mode mode_ = mode::OFF;
  static inline struct k_work_delayable blink_work_ = {};
};

}  // namespace uwc
