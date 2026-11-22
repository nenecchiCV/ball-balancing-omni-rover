#include "feature/link_monitor/link_monitor.hpp"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "peripheral/status_led/status_led.hpp"

namespace uwc {

void link_monitor::init() {
  stats_ = {};
  connected_ = false;
}

void link_monitor::on_event(link::event ev) {
  const unsigned int key = irq_lock();
  switch (ev) {
    case link::event::CONNECTED:
      connected_ = true;
      ++stats_.connect_count;
      break;
    case link::event::DISCONNECTED:
      connected_ = false;
      ++stats_.disconnect_count;
      break;
    default:
      break;
  }
  irq_unlock(key);

  // リンク状態をユーザー LED に反映する（FR-05）
  switch (ev) {
    case link::event::WAITING_STARTED:
      status_led::set_mode(status_led::mode::BLINK);
      break;
    case link::event::CONNECTED:
      status_led::set_mode(status_led::mode::SOLID);
      break;
    case link::event::DISCONNECTED:
      status_led::set_mode(status_led::mode::OFF);
      break;
    default:
      break;
  }
}

void link_monitor::note_tx(uint32_t bytes) {
  const unsigned int key = irq_lock();
  stats_.bytes_host_to_link += bytes;
  stats_.last_activity_ms = k_uptime_get_32();
  irq_unlock(key);
}

void link_monitor::note_rx(uint32_t bytes) {
  const unsigned int key = irq_lock();
  stats_.bytes_link_to_host += bytes;
  stats_.last_activity_ms = k_uptime_get_32();
  irq_unlock(key);
}

void link_monitor::note_drop(uint32_t bytes) {
  const unsigned int key = irq_lock();
  stats_.drop_bytes += bytes;
  irq_unlock(key);
}

void link_monitor::note_send_fail() {
  const unsigned int key = irq_lock();
  ++stats_.send_fail_count;
  irq_unlock(key);
}

link_monitor::stats link_monitor::snapshot() {
  stats s{};
  const unsigned int key = irq_lock();
  s = stats_;
  irq_unlock(key);
  return s;
}

void link_monitor::reset() {
  const unsigned int key = irq_lock();
  const bool keep = connected_;
  stats_ = {};
  connected_ = keep;
  irq_unlock(key);
}

bool link_monitor::connected() {
  return connected_;
}

}  // namespace uwc
