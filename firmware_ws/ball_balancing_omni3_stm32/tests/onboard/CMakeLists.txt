cmake_minimum_required(VERSION 3.20.0)

# 実機ソフトと同じ devicetree 構成（bb,pins / qdec / imu / timers6）を適用する。
set(DTC_OVERLAY_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../boards/nucleo_g474re.overlay")

# 実機ソフト側の dts/bindings（bb,pins 等）を参照する。
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# hal_stm32 モジュール（west ワークスペース外のため明示）。
if(NOT DEFINED ENV{ZEPHYR_WS_DIR})
  set(ZEPHYR_WS_DIR "C:/zephyr_ws")
else()
  set(ZEPHYR_WS_DIR "$ENV{ZEPHYR_WS_DIR}")
endif()
list(APPEND ZEPHYR_MODULES
  ${ZEPHYR_WS_DIR}/modules/hal/stm32
  ${ZEPHYR_WS_DIR}/modules/hal/cmsis_6
  ${ZEPHYR_WS_DIR}/modules/hal/st
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bb_onboard_tests LANGUAGES C CXX)

set(APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_include_directories(app PRIVATE
  ${APP_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/etl/include
)

# 制限付きC++: 例外・RTTI・動的確保は使用しない。
target_compile_options(app PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics -Wall -Wextra -DETL_NO_STL>
)

target_sources(app PRIVATE
  src/main.cpp

  ${APP_SRC}/peripheral/gpio_pin/gpio_pin.cpp
  ${APP_SRC}/peripheral/pwm_output/pwm_output.cpp
  ${APP_SRC}/peripheral/encoder_counter/encoder_counter.cpp
  ${APP_SRC}/peripheral/timebase/timebase.cpp
  ${APP_SRC}/peripheral/control_tick/control_tick.cpp
  ${APP_SRC}/peripheral/debug_uart/debug_uart.cpp

  ${APP_SRC}/device/lsm6dso/lsm6dso.cpp
  ${APP_SRC}/device/wheel_encoder/wheel_encoder.cpp
  ${APP_SRC}/device/mdd3a_motor/mdd3a_motor.cpp
  ${APP_SRC}/device/status_led/status_led.cpp
  ${APP_SRC}/device/estop_input/estop_input.cpp

  ${APP_SRC}/feature/state_estimator/state_estimator.cpp
  ${APP_SRC}/feature/wheel_speed_meter/wheel_speed_meter.cpp

  src/test_application/test_application.cpp
  src/test_runner/test_runner.cpp
  src/estop_test/estop_test.cpp
  src/tick_timing_test/tick_timing_test.cpp
  src/imu_attitude_test/imu_attitude_test.cpp
  src/encoder_test/encoder_test.cpp
  src/omni_drive_test/omni_drive_test.cpp
  src/motor_deadzone_test/motor_deadzone_test.cpp
  src/uart_echo_test/uart_echo_test.cpp
)
