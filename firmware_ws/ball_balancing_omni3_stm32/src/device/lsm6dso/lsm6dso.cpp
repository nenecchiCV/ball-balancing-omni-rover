#include "device/lsm6dso/lsm6dso.hpp"

namespace bb {

bool lsm6dso::init(const config& cfg) {
  dev_ = cfg.dev;
  axis_map_ = cfg.axis_map;
  return device_is_ready(dev_);
}

lsm6dso::sample lsm6dso::read() {
  sample s{};
  struct sensor_value accel[3]{};
  struct sensor_value gyro[3]{};
  if (sensor_sample_fetch(dev_) != 0) return s;
  if (sensor_channel_get(dev_, SENSOR_CHAN_ACCEL_XYZ, accel) != 0 ||
      sensor_channel_get(dev_, SENSOR_CHAN_GYRO_XYZ, gyro) != 0) {
    return s;
  }
  const vec3 accel_raw = {
      static_cast<float>(sensor_value_to_double(&accel[0])),
      static_cast<float>(sensor_value_to_double(&accel[1])),
      static_cast<float>(sensor_value_to_double(&accel[2]))};
  const vec3 gyro_raw = {static_cast<float>(sensor_value_to_double(&gyro[0])),
                         static_cast<float>(sensor_value_to_double(&gyro[1])),
                         static_cast<float>(sensor_value_to_double(&gyro[2]))};
  s.gyro = mul(axis_map_, gyro_raw);
  s.specific_force = mul(axis_map_, accel_raw);
  s.valid = true;
  return s;
}

}  // namespace bb
