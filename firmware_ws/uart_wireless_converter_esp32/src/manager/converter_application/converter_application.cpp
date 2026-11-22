#include "manager/converter_application/converter_application.hpp"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#include "device/link/link.hpp"
#include "feature/link_monitor/link_monitor.hpp"
#include "feature/uart_bridge/uart_bridge.hpp"
#include "peripheral/debug_console/debug_console.hpp"
#include "peripheral/host_uart/host_uart.hpp"
#include "peripheral/status_led/status_led.hpp"

namespace uwc {

namespace {
constexpr uint32_t BRIDGE_POLL_MS = 1;
// ユーザー LED（led0、GPIO15）。ボード dts の alias をそのまま使用する。
const struct gpio_dt_spec LED0_SPEC = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
}  // namespace

bool converter_application::init(const config& cfg) {
  ARG_UNUSED(cfg);
  ring_buf_init(&host_to_link_, sizeof(h2l_mem_), h2l_mem_);
  ring_buf_init(&link_to_host_, sizeof(l2h_mem_), l2h_mem_);

  link_monitor::init();

  const status_led::config led_cfg = {.led = &LED0_SPEC};
  if (!status_led::init(led_cfg)) {
    debug_console::write_line("uwc: status_led init failed");
    return false;
  }

  if (!uart_bridge::init({&host_to_link_, &link_to_host_, BRIDGE_POLL_MS})) {
    debug_console::write_line("uwc: bridge init failed");
    return false;
  }

  const struct device* uart_dev = DEVICE_DT_GET(DT_ALIAS(host_uart));
  const host_uart::config uart_cfg = {
      .uart_dev = uart_dev,
      .rx_ring = &host_to_link_,
      .rx_sem = uart_bridge::rx_sem(),
  };
  if (!host_uart::init(uart_cfg)) {
    debug_console::write_line("uwc: host_uart init failed");
    return false;
  }

  const link::config link_cfg = {
      .rx_ring = &link_to_host_,
      .rx_sem = uart_bridge::rx_sem(),
      .event_cb = &link_monitor::on_event,
  };
  if (!link::init(link_cfg)) {
    debug_console::write_line("uwc: link init failed");
    return false;
  }

  debug_console::write_line("uwc: ready");
  return true;
}

void converter_application::run() {
  uart_bridge::run();
}

link_monitor::stats converter_application::stats() {
  return link_monitor::snapshot();
}

void converter_application::reset_stats() {
  link_monitor::reset();
}

}  // namespace uwc

namespace {

int status_cmd_cb(const struct shell* shell, size_t argc, char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  char desc[64];
  const uwc::link_monitor::stats s = uwc::converter_application::stats();
  shell_print(shell, "link: %s", uwc::link::describe(desc, sizeof(desc)));
  shell_print(
      shell, "state: %s connected=%s last_activity=%ums",
      (uwc::link::link_state() == uwc::link::state::CONNECTED) ? "connected"
      : (uwc::link::link_state() == uwc::link::state::WAITING) ? "waiting"
                                                               : "idle",
      s.connect_count > s.disconnect_count ? "yes" : "no",
      static_cast<unsigned>(s.last_activity_ms));
  return 0;
}

int stats_cmd_cb(const struct shell* shell, size_t argc, char** argv) {
  if (argc > 1 && strcmp(argv[1], "reset") == 0) {
    uwc::converter_application::reset_stats();
    shell_print(shell, "stats reset");
    return 0;
  }
  const uwc::link_monitor::stats s = uwc::converter_application::stats();
  shell_print(shell, "host->link: %u B  link->host: %u B",
              static_cast<unsigned>(s.bytes_host_to_link),
              static_cast<unsigned>(s.bytes_link_to_host));
  shell_print(shell, "send_fail=%u drop=%u B (uart_rx=%u link_rx=%u)",
              static_cast<unsigned>(s.send_fail_count),
              static_cast<unsigned>(s.drop_bytes),
              static_cast<unsigned>(uwc::host_uart::rx_dropped()),
              static_cast<unsigned>(uwc::link::rx_dropped()));
  shell_print(shell, "connect=%u disconnect=%u",
              static_cast<unsigned>(s.connect_count),
              static_cast<unsigned>(s.disconnect_count));
  return 0;
}

// シェルマクロの ?: は nullptr_t と int の混在を許さないため NULL を渡す。
// NOLINTBEGIN: Zephyr の SHELL_* マクロ内部の初期化子・分岐に由来する警告
SHELL_STATIC_SUBCMD_SET_CREATE(
    uwc_cmds, SHELL_CMD(status, NULL, "show link status", status_cmd_cb),
    SHELL_CMD_ARG(stats, NULL, "stats [reset]", stats_cmd_cb, 1, 1));
SHELL_CMD_REGISTER(uwc, &uwc_cmds, "uart-wireless converter", NULL);
// NOLINTEND

}  // namespace
