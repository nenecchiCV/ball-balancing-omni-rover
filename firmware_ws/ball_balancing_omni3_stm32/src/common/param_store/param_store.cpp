#include "common/param_store/param_store.hpp"

#include <etl/algorithm.h>
#include <math.h>
#include <string.h>

namespace bb {

const param_set& get_params() {
  return param_store::params();
}

namespace {

constexpr uint8_t GAIN_KEY_BASE = 0x30;  // torque_lqi.state_gain[r][c] = 0x30+r*10+c

}  // namespace

void param_store::init_defaults() {
  params_ = param_set{};
  save_seq_ = 0;
  derive();

  // key → フィールド変換テーブル（key 昇順、調整対象はプロトコル正本に準拠）。
  uint32_t i = 0;
  const auto reg = [&](uint8_t key, float* f, float mn, float mx,
                     bool ro = false) {
    entries_[i] = entry{key, f, mn, mx, ro};
    ++i;
  };
  for (int w = 0; w < 3; ++w) {
    reg(static_cast<uint8_t>(0x00 + w), &params_.drive.deadzone_comp_pos[w],
        0.0F, 0.3F);
  }
  for (int w = 0; w < 3; ++w) {
    reg(static_cast<uint8_t>(0x03 + w), &params_.drive.deadzone_comp_neg[w],
        0.0F, 0.3F);
  }
  reg(0x10, &params_.lqi.tracking_gain_scale, 0.1F, 3.0F);
  reg(0x11, &params_.lqi.anti_windup_gain, 0.0F, 20.0F);
  reg(0x12, &params_.torque_drive.yaw_damping_gain, 0.0F, 1.0F);
  reg(0x13, &params_.torque_drive.tilt_integral_gain, 0.0F, 2.0F);
  reg(0x20, &params_.position.kp, 0.0F, 5.0F);
  reg(0x21, &params_.position.accel_limit, 0.0F, 2.0F);
  reg(0x22, &params_.position.arrive_radius, 0.005F, 0.2F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 10; ++c) {
      reg(static_cast<uint8_t>(GAIN_KEY_BASE + r * 10 + c),
          &params_.torque_lqi.state_gain[r][c], -50.0F, 50.0F);
    }
  }
  // key 昇順を保つため IMU 特性は gain 表（0x30–0x4D）の後ろに置く。
  reg(0x50, &params_.imu.gyro_bias_z, -0.2F, 0.2F);
  reg(0x51, &params_.imu.ekf_bias_x, -0.2F, 0.2F);
  reg(0x52, &params_.imu.ekf_bias_y, -0.2F, 0.2F);
  // 速度上限は直接設定値（初期値 0.4 = シム検証値）。設定可能上限は
  // 無滑り理論上限で、0x61 はその派生値の読み取り専用参照。
  reg(0x60, &params_.controller.max_speed, 0.05F, theory_max_speed_);
  reg(0x61, &theory_max_speed_, 0.0F, 0.0F, true);
}

const param_set& param_store::params() {
  return params_;
}

param_set& param_store::mutable_params() {
  return params_;
}

param_store::entry* param_store::find(uint8_t key) {
  for (auto& e : entries_) {
    if (e.key == key) {
      return &e;
    }
  }
  return nullptr;
}

bool param_store::get_key(uint8_t key, float* value) {
  const entry* const e = find(key);
  if (e == nullptr || value == nullptr) {
    return false;
  }
  *value = *e->field;
  return true;
}

bool param_store::set_key(uint8_t key, float value) {
  const entry* const e = find(key);
  if (e == nullptr || e->read_only || value < e->min || value > e->max) {
    return false;
  }
  *e->field = value;
  return true;
}

uint8_t param_store::key_at(uint32_t index) {
  return (index < KEY_COUNT) ? entries_[index].key : 0xFF;
}

float param_store::theoretical_max_speed() {
  return theory_max_speed_;
}

void param_store::derive() {
  const float s = sinf(params_.physical.contact_latitude);
  theory_max_speed_ =
      (s > 1e-6F)
          ? params_.physical.wheel_radius *
                params_.motor.speed_command_limit / s
          : 0.0F;
  // blob 適用で物理定数が変わり得るため、0x60 の設定可能上限と
  // 現値も新しい理論上限へ合わせる（init_defaults では未登録で無害）。
  if (entry* const e = find(0x60)) {
    e->max = theory_max_speed_;
    params_.controller.max_speed =
        etl::min(params_.controller.max_speed, theory_max_speed_);
  }
}

void param_store::build_blob(uint32_t seq, uint8_t* out) {
  // magic
  uint32_t magic = BLOB_MAGIC;
  memcpy(out, &magic, 4);
  uint16_t ver = BLOB_SCHEMA_VER;
  memcpy(&out[4], &ver, 2);
  memcpy(&out[6], &seq, 4);
  memcpy(&out[BLOB_HEADER], &params_, sizeof(params_));
  const uint32_t crc = crc32(out, BLOB_HEADER + sizeof(params_));
  memcpy(&out[BLOB_HEADER + sizeof(params_)], &crc, 4);
}

bool param_store::parse_blob(const uint8_t* data, uint32_t len,
                             uint32_t* seq) {
  if (len < BLOB_SIZE) {
    return false;
  }
  uint32_t magic = 0;
  memcpy(&magic, data, 4);
  uint16_t ver = 0;
  memcpy(&ver, &data[4], 2);
  if (magic != BLOB_MAGIC || ver != BLOB_SCHEMA_VER) {
    return false;
  }
  uint32_t crc = 0;
  memcpy(&crc, &data[BLOB_HEADER + sizeof(params_)], 4);
  if (crc != crc32(data, BLOB_HEADER + sizeof(params_))) {
    return false;
  }
  memcpy(&params_, &data[BLOB_HEADER], sizeof(params_));
  derive();
  uint32_t loaded_seq = 0;
  memcpy(&loaded_seq, &data[6], 4);
  save_seq_ = etl::max(save_seq_, loaded_seq);
  if (seq != nullptr) {
    *seq = loaded_seq;
  }
  return true;
}

uint32_t param_store::next_save_seq() {
  return ++save_seq_;
}

uint32_t param_store::crc32(const uint8_t* data, uint32_t len) {
  // CRC-32（poly 0xEDB88320, init/xorout 0xFFFFFFFF）。
  uint32_t crc = 0xFFFFFFFFU;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 1U) != 0 ? (crc >> 1) ^ 0xEDB88320U : crc >> 1;
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

}  // namespace bb
