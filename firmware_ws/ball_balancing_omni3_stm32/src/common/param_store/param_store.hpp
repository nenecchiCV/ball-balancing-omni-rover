#pragma once

#include <cstdint>

#include "common/control_params/control_params.hpp"

namespace bb {

// 動作パラメータ（param_set の RAM 上の可変コピー）の保持・既定値初期化・
// key 単位 get/set・フラッシュ保存 blob の符号化/復号（FR-15, FR-17）。
// フラッシュ自体の読み書きは peripheral/flash_store が担い、
// 本コンポーネントは純粋ロジック（ホストテスト対象）とする。
class param_store {
 public:
  param_store() = delete;

  // 保存フォーマット（LE）:
  //   magic u32 | schema_ver u16 | seq u32 | param_set | crc32 u32
  // crc32 は magic 先頭から param_set 末尾までの区間に掛ける。
  static constexpr uint32_t BLOB_MAGIC = 0x42425041U;  // "BBPA"
  static constexpr uint16_t BLOB_SCHEMA_VER = 3;
  static constexpr uint32_t BLOB_HEADER = 4 + 2 + 4;
  // STM32 のフラッシュ書込みは 8 B（double word）単位必須のため、
  // 末尾パディングで総サイズを 8 の倍数に揃える（パディングは未使用）。
  static constexpr uint32_t BLOB_SIZE =
      (BLOB_HEADER + sizeof(param_set) + 4 + 7) & ~7U;

  static void init_defaults();          // 動作値を constexpr 既定値へ初期化
  static const param_set& params();     // 動作値（get_params() の実体）
  static param_set& mutable_params();

  // key 単位アクセス（調整対象表は robot_link_protocol.md「調整対象パラメータ」）。
  static constexpr uint32_t KEY_COUNT = 48;
  static bool get_key(uint8_t key, float* value);
  static bool set_key(uint8_t key, float value);  // 未定義/範囲外/読取専用は false
  static uint8_t key_at(uint32_t index);          // 昇順列挙（index < KEY_COUNT）
  // 派生値: 無滑り理論速度上限 = wheel_radius*speed_command_limit/sin(lat)。
  static float theoretical_max_speed();

  static uint32_t blob_size() {
    return BLOB_SIZE;
  }
  // out（blob_size() バイト以上）へ保存 blob を書く。
  static void build_blob(uint32_t seq, uint8_t* out);
  // blob を検証し、妥当なら param_set を動作値へ適用して seq を返す。
  // magic/schema_ver/crc32 不整合・長不足は false で動作値を維持する。
  static bool parse_blob(const uint8_t* data, uint32_t len, uint32_t* seq);
  static uint32_t crc32(const uint8_t* data, uint32_t len);

 private:
  struct entry {
    uint8_t key;
    float* field;
    float min;
    float max;
    bool read_only;
  };

  static entry* find(uint8_t key);
  static void derive();  // 派生値（理論速度上限・0x60 上限）を params_ から更新

  static inline param_set params_{};
  static inline entry entries_[KEY_COUNT]{};
  static inline float theory_max_speed_ = 0.0F;
};

}  // namespace bb
