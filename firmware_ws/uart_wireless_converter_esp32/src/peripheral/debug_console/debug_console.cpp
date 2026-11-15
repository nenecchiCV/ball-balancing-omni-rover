#include "peripheral/debug_console/debug_console.hpp"

#include <zephyr/sys/printk.h>

namespace uwc {

void debug_console::write(const char* text, uint32_t len) {
  for (uint32_t i = 0; i < len; ++i) printk("%c", text[i]);
}

void debug_console::write_line(const char* text) {
  printk("%s\n", text);
}

}  // namespace uwc
