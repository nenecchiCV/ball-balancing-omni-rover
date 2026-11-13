#include "imu_attitude_test/imu_attitude_test.hpp"

#include <stdio.h>

#include "device/lsm6dso/lsm6dso.hpp"
#include "feature/state_estimator/state_estimator.hpp"
#include "peripheral/debug_uart/debug_uart.hpp"
#include "peripheral/timebase/timebase.hpp"

namespace bb {

namespace {
// 200 Hz 更新のうち表示は 10 Hz。
constexpr uint32_t PRINT_EVERY = 20U;
constexpr float RAD2DEG = 57.29577951308232F;
}  // namespace

void imu_attitude_test::init(const config& cfg) {
  p_ = cfg.params;
}

bool imu_attitude_test::start_cb(int argc, char** argv) {
  (void)argc;
  (void)argv;
  debug_uart::write_line(
      "imu test: tilt the chassis; check euler_deg matches the physical "
      "roll/pitch/yaw");
  debug_uart::write_line(
      "level -> euler~[0 0 *]; tilt +x-forward -> pitch-; tilt +y-left -> "
      "roll+ (axis_map dependent)");
  print_phase_ = 0;
  return true;
}

void imu_attitude_test::update_cb() {
  // 5 tick 周期 = 200 Hz。推定器は製品と同じ外側周期 dt で回す。
  const lsm6dso::sample imu = lsm6dso::read();
  const vec3 wheel_rate{};  // IMU 単体検証のため輪速はゼロ入力。
  const state_estimator::output est =
      state_estimator::step(imu, wheel_rate, p_->controller.sample_time);

  if (++print_phase_ < PRINT_EVERY) return;
  print_phase_ = 0;

  char line[200];
  const int n = snprintf(line, sizeof(line),
                         "imu t=%lu acc=[%.3f %.3f %.3f] gyr=[%.3f %.3f %.3f] "
                         "euler_deg=[%.2f %.2f %.2f] valid=%d\r\n",
                         static_cast<unsigned long>(timebase::millis()),
                         static_cast<double>(imu.specific_force[0]),
                         static_cast<double>(imu.specific_force[1]),
                         static_cast<double>(imu.specific_force[2]),
                         static_cast<double>(imu.gyro[0]),
                         static_cast<double>(imu.gyro[1]),
                         static_cast<double>(imu.gyro[2]),
                         static_cast<double>(est.euler[0] * RAD2DEG),
                         static_cast<double>(est.euler[1] * RAD2DEG),
                         static_cast<double>(est.euler[2] * RAD2DEG),
                         static_cast<int>(imu.valid && est.valid));
  debug_uart::write(line, static_cast<uint32_t>(n));
}

void imu_attitude_test::stop_cb() {
  debug_uart::write_line("imu test: stopped");
}

}  // namespace bb
