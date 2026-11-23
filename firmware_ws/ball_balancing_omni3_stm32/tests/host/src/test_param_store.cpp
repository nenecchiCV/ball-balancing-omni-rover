// param_store ホストテスト: 既定値・key get/set・blob 符号化/復号（FR-15/17）。
#include <math.h>
#include <string.h>

#include "common/param_store/param_store.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

void defaults_match_constexpr() {
  param_store::init_defaults();
  const param_set ref{};
  CHECK_NEAR(param_store::params().drive.deadzone_comp[0],
             ref.drive.deadzone_comp[0], 1e-9F);
  CHECK_NEAR(param_store::params().lqi.tracking_gain_scale,
             ref.lqi.tracking_gain_scale, 1e-9F);
  CHECK_NEAR(param_store::params().imu.gyro_bias_z, ref.imu.gyro_bias_z,
             1e-9F);
}

void key_roundtrip() {
  param_store::init_defaults();
  // deadzone_comp[1] = key 0x01
  CHECK_TRUE(param_store::set_key(0x01, 0.12F));
  float v = 0.0F;
  CHECK_TRUE(param_store::get_key(0x01, &v));
  CHECK_NEAR(v, 0.12F, 1e-9F);
  // 範囲外は拒否
  CHECK_TRUE(!param_store::set_key(0x01, 0.5F));
  CHECK_TRUE(param_store::get_key(0x01, &v));
  CHECK_NEAR(v, 0.12F, 1e-9F);
  // 未定義キーは拒否
  CHECK_TRUE(!param_store::set_key(0xEE, 1.0F));
  CHECK_TRUE(!param_store::get_key(0xEE, &v));
}

void key_table_covers_gain_matrix() {
  param_store::init_defaults();
  // torque_lqi.state_gain[2][9] = 0x30 + 2*10 + 9 = 0x4D
  const param_set ref{};
  const float want = ref.torque_lqi.state_gain[2][9];
  float v = 0.0F;
  CHECK_TRUE(param_store::get_key(0x4D, &v));
  CHECK_NEAR(v, want, 1e-9F);
  CHECK_TRUE(param_store::set_key(0x4D, 1.5F));
  CHECK_NEAR(param_store::params().torque_lqi.state_gain[2][9], 1.5F, 1e-9F);
}

void key_at_is_sorted() {
  param_store::init_defaults();
  uint8_t prev = 0;
  for (uint32_t i = 0; i < param_store::KEY_COUNT; ++i) {
    const uint8_t k = param_store::key_at(i);
    CHECK_TRUE(k >= prev || i == 0);
    prev = k;
  }
  CHECK_TRUE(param_store::key_at(param_store::KEY_COUNT) == 0xFF);
}

void blob_roundtrip() {
  param_store::init_defaults();
  CHECK_TRUE(param_store::set_key(0x50, 0.01F));  // imu.gyro_bias_z
  static uint8_t blob[param_store::BLOB_SIZE];
  param_store::build_blob(7, blob);

  // 別値へ変えてから復号すると保存値に戻る。
  CHECK_TRUE(param_store::set_key(0x50, -0.05F));
  uint32_t seq = 0;
  CHECK_TRUE(param_store::parse_blob(blob, param_store::BLOB_SIZE, &seq));
  CHECK_TRUE(seq == 7);
  CHECK_NEAR(param_store::params().imu.gyro_bias_z, 0.01F, 1e-9F);
}

void blob_rejects_corruption() {
  param_store::init_defaults();
  static uint8_t blob[param_store::BLOB_SIZE];
  param_store::build_blob(3, blob);
  // CRC 破損
  uint8_t bad[param_store::BLOB_SIZE];
  memcpy(bad, blob, sizeof(bad));
  bad[param_store::BLOB_SIZE - 1] ^= 0xFF;
  CHECK_TRUE(!param_store::parse_blob(bad, sizeof(bad), nullptr));
  // magic 不整合
  memcpy(bad, blob, sizeof(bad));
  bad[0] ^= 0xFF;
  CHECK_TRUE(!param_store::parse_blob(bad, sizeof(bad), nullptr));
  // 長不足
  CHECK_TRUE(!param_store::parse_blob(blob, 8, nullptr));
}

void speed_limit_key() {
  param_store::init_defaults();
  // 理論上限 = wheel_radius*speed_command_limit/sin(contact_latitude)
  const param_set ref{};
  const float theory = ref.physical.wheel_radius *
                       ref.motor.speed_command_limit /
                       sinf(ref.physical.contact_latitude);
  CHECK_NEAR(param_store::theoretical_max_speed(), theory, 1e-6F);
  float v = 0.0F;
  CHECK_TRUE(param_store::get_key(0x61, &v));
  CHECK_NEAR(v, theory, 1e-6F);
  // 既定 max_speed = 0.4（シム検証値）、理論上限（≈0.81）まで設定可。
  CHECK_TRUE(param_store::get_key(0x60, &v));
  CHECK_NEAR(v, 0.4F, 1e-6F);
  CHECK_TRUE(param_store::set_key(0x60, 0.8F));
  CHECK_TRUE(!param_store::set_key(0x60, 0.9F));
  CHECK_TRUE(!param_store::set_key(0x60, 0.0F));
  // 読み取り専用 key は set 不可・get 可。
  CHECK_TRUE(!param_store::set_key(0x61, 1.0F));
  CHECK_TRUE(param_store::get_key(0x61, &v));
}

void crc32_known_vector() {
  // CRC-32("123456789") = 0xCBF43926。
  const uint8_t s[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK_TRUE(param_store::crc32(s, 9) == 0xCBF43926U);
}

}  // namespace

void param_store_tests() {
  defaults_match_constexpr();
  key_roundtrip();
  key_table_covers_gain_matrix();
  key_at_is_sorted();
  blob_roundtrip();
  blob_rejects_corruption();
  crc32_known_vector();
}

}  // namespace bb
