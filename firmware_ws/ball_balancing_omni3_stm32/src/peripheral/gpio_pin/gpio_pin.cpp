#include "peripheral/gpio_pin/gpio_pin.hpp"

namespace bb {

bool gpio_pin::init(const config& cfg) {
  config_ = cfg;
  if (!gpio_is_ready_dt(&config_.spec)) return false;
  gpio_flags_t mode = GPIO_INPUT;
  if (config_.output) {
    mode = config_.initial_high ? GPIO_OUTPUT_HIGH : GPIO_OUTPUT_LOW;
  }
  return gpio_pin_configure_dt(&config_.spec, mode) == 0;
}

bool gpio_pin::read() const {
  return gpio_pin_get_dt(&config_.spec) > 0;
}

void gpio_pin::write(bool level) const {
  gpio_pin_set_dt(&config_.spec, level ? 1 : 0);
}

bool gpio_pin::init_interrupt(edge e) {
  gpio_flags_t flags = GPIO_INT_EDGE_BOTH;
  if (e == edge::FALLING) flags = GPIO_INT_EDGE_FALLING;
  if (e == edge::RISING) flags = GPIO_INT_EDGE_RISING;
  if (gpio_pin_interrupt_configure_dt(&config_.spec, flags) != 0) return false;
  gpio_init_callback(&callback_, gpio_irq_cb, BIT(config_.spec.pin));
  return gpio_add_callback_dt(&config_.spec, &callback_) == 0;
}

bool gpio_pin::consume_irq_flag() {
  const bool pending = irq_pending_;
  irq_pending_ = false;
  return pending;
}

void gpio_pin::gpio_irq_cb(const struct device* port, struct gpio_callback* cb,
                           gpio_port_pins_t pins) {
  ARG_UNUSED(port);
  ARG_UNUSED(pins);
  auto* self = CONTAINER_OF(cb, gpio_pin, callback_);
  self->irq_pending_ = true;
}

}  // namespace bb
