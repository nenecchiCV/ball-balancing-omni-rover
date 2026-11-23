#pragma once

#include <cstdint>

namespace bb {

// storage_partition（4 KiB）を 2 スロットの不揮発パラメータ保存領域として
// 読み書きする（FR-15）。blob 形式の検証は common/param_store が担い、
// 本コンポーネントはデバイス依存の読み書きのみを提供する。
class flash_store {
 public:
  flash_store() = delete;

  static constexpr uint32_t SLOT_COUNT = 2;
  static constexpr uint32_t SLOT_SIZE = 2048;  // 消去ページ境界（2 KiB）揃え

  // 2 スロットを走査し、seq 最大の妥当 blob を out（data_len 以上）へ読み込む。
  // 妥当スロットが 1 つもなければ false。seq_out には採用 blob の seq を返す。
  static bool load_latest(uint8_t* out, uint32_t data_len, uint32_t* seq_out);
  // slot を 1 ページ消去して data（data_len ≤ SLOT_SIZE）を書き込む。
  // 書き込み検証込み。ブロッキング（消去+書込み+検証で 100 ms 級）。
  static bool write_slot(uint32_t slot, const uint8_t* data, uint32_t data_len);
  // 両スロットを消去（保存値のリセット）。
  static bool erase_all();

 private:
  static bool open_area();
  static uint32_t slot_offset(uint32_t slot) {
    return slot * SLOT_SIZE;
  }
};

}  // namespace bb
