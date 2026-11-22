cmake_minimum_required(VERSION 3.20.0)

# モジュール（hal_espressif, mbedtls 等）は west マニフェストから自動検出する。
# 無線（BLE/Wi-Fi）利用には hal_espressif と `west blobs fetch hal_espressif`
# で取得するバイナリ blob が必要。
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart_wireless_converter_esp32 LANGUAGES C CXX)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/etl/include
)

# 制限付きC++: 例外・RTTI・動的確保は使用しない。
# ただし ESP32 無線スタック（hal_espressif blob）の内部確保のため
# カーネルヒープ自体は有効化する（NFR-04 参照）。
target_compile_options(app PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics -Wall -Wextra -DETL_NO_STL>
)

target_sources(app PRIVATE
  src/main.cpp

  src/peripheral/host_uart/host_uart.cpp
  src/peripheral/debug_console/debug_console.cpp
  src/peripheral/status_led/status_led.cpp

  src/feature/uart_bridge/uart_bridge.cpp
  src/feature/link_monitor/link_monitor.cpp

  src/manager/converter_application/converter_application.cpp
)

# トランスポートは Kconfig (UWC_TRANSPORT_*) で選択する（FR-04, NFR-03）。
# 同一クラス名 `link` の実装をビルド時に切替える。
if(CONFIG_UWC_TRANSPORT_WIFI)
  target_sources(app PRIVATE src/device/link/link_wifi.cpp)
else()
  target_sources(app PRIVATE src/device/link/link_ble.cpp)
endif()
