# feature 層のホスト単体テスト（Zephyr 非依存、ホスト clang++ で直接ビルド）。
# 使い方:
#   cmake -B build -G Ninja -DCMAKE_CXX_COMPILER=clang++ -S tests/host
#   cmake --build build
#   ctest --test-dir build --output-on-failure
cmake_minimum_required(VERSION 3.20.0)
project(bb_feature_host_tests CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 制限付きC++: 例外・RTTI・動的確保・実行時 static 初期化ガードなし
add_compile_options(-fno-exceptions -fno-rtti -fno-threadsafe-statics -Wall
                    -Wextra -DETL_NO_STL)

get_filename_component(FW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

add_executable(
  bb_feature_tests
  ${FW_ROOT}/src/common/link_protocol/link_protocol.cpp
  ${FW_ROOT}/src/feature/balance_controller/balance_controller.cpp
  ${FW_ROOT}/src/feature/command_profile/command_profile.cpp
  ${FW_ROOT}/src/feature/odometry/odometry.cpp
  ${FW_ROOT}/src/feature/position_controller/position_controller.cpp
  ${FW_ROOT}/src/feature/state_estimator/state_estimator.cpp
  ${FW_ROOT}/src/feature/wheel_servo/wheel_servo.cpp
  src/test_balance_controller.cpp
  src/test_command_profile.cpp
  src/test_link_protocol.cpp
  src/test_main.cpp
  src/test_odometry.cpp
  src/test_position_controller.cpp
  src/test_state_estimator.cpp
  src/test_wheel_servo.cpp)

# stubs/ には <zephyr/irq.h> 等のホスト用スタブを置く（実ファイル側に
# 同名ヘッダが無いため解決順は問わない）。
target_include_directories(bb_feature_tests PRIVATE stubs ${FW_ROOT}/src
                                                    ${FW_ROOT}/third_party/etl/include src)

enable_testing()
add_test(NAME bb_feature_tests COMMAND bb_feature_tests)
