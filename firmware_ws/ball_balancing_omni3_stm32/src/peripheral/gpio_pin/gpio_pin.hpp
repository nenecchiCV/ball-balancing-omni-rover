#pragma once

#include <zephyr/drivers/gpio.h>

namespace bb {

// GPIO ピン（devicetree の gpio_dt_spec を保持する薄いアダプタ）。
// 方向・初期レベルは init() 時に設定し、EXTI 割込は init_interrupt()
// で登録する。複数インスタンスが必要なためクラスのままとし、
// コンストラクタではなく init() で設定を受け取る。
class gpio_pin {
 public:
  enum class edge : uint8_t { FALLING, RISING, BOTH };

  struct config {
    gpio_dt_spec spec = {};
    bool output = false;
    bool initial_high = false;
  };

  bool init(const config& cfg);
  [[nodiscard]] bool read() const;
  void write(bool level) const;
  // 割込検知（estop 用）。edge に応じて EXTI を有効化し、
  // フラグは consume_irq_flag() で取得・クリアする。
  bool init_interrupt(edge e);
  bool consume_irq_flag();

 private:
  static void gpio_irq_cb(const struct device* port, struct gpio_callback* cb,
                          gpio_port_pins_t pins);

  config config_{};
  struct gpio_callback callback_{};
  volatile bool irq_pending_ = false;
};

}  // namespace bb
