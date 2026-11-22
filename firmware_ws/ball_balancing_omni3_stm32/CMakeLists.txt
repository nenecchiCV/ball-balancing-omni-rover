cmake_minimum_required(VERSION 3.20.0)

# hal_stm32 モジュール（west ワークスペース外のため明示）。
# 既定は C:/zephyr_ws、環境変数 ZEPHYR_WS_DIR で上書き可能。
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
project(ball_balancing_omni3_stm32 LANGUAGES C CXX)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/etl/include
)

# 制限付きC++: 例外・RTTI・動的確保は使用しない。
target_compile_options(app PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics -Wall -Wextra -DETL_NO_STL>
)

target_sources(app PRIVATE
  src/main.cpp


  src/peripheral/gpio_pin/gpio_pin.cpp
  src/peripheral/pwm_output/pwm_output.cpp
  src/peripheral/encoder_counter/encoder_counter.cpp
  src/peripheral/timebase/timebase.cpp
  src/peripheral/control_tick/control_tick.cpp
  src/peripheral/debug_uart/debug_uart.cpp
  src/peripheral/link_uart/link_uart.cpp

  src/common/link_protocol/link_protocol.cpp

  src/device/lsm6dso/lsm6dso.cpp
  src/device/wheel_encoder/wheel_encoder.cpp
  src/device/mdd3a_motor/mdd3a_motor.cpp
  src/device/status_led/status_led.cpp
  src/device/estop_input/estop_input.cpp

  src/feature/state_estimator/state_estimator.cpp
  src/feature/balance_controller/balance_controller.cpp
  src/feature/wheel_servo/wheel_servo.cpp
  src/feature/command_profile/command_profile.cpp
  src/feature/wheel_speed_meter/wheel_speed_meter.cpp
  src/feature/odometry/odometry.cpp
  src/feature/position_controller/position_controller.cpp
  src/feature/link_session/link_session.cpp

  src/manager/robot_application/robot_application.cpp
  src/manager/task_scheduler/task_scheduler.cpp
)
