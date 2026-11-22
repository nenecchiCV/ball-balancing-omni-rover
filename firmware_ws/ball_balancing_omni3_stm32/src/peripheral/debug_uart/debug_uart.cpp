#include "peripheral/debug_uart/debug_uart.hpp"

#include <zephyr/sys/printk.h>

namespace bb {

void debug_uart::write(const char* text, uint32_t len) {
  printk("%.*s", static_cast<int>(len), text);
}

void debug_uart::write_line(const char* text) {
  printk("%s\n", text);
}

}  // namespace bb
