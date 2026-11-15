#include <zephyr/kernel.h>

#include "manager/converter_application/converter_application.hpp"

int main() {
  const uwc::converter_application::config cfg{};
  if (!uwc::converter_application::init(cfg)) {
    return -1;
  }
  uwc::converter_application::run();  // 戻らない
}
