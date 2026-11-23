#include "peripheral/flash_store/flash_store.hpp"

#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/devicetree/fixed-partitions.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>

#include "common/param_store/param_store.hpp"

#if !DT_NODE_EXISTS(DT_NODELABEL(storage_partition))
#error "storage_partition is required for parameter persistence"
#endif

namespace bb {

namespace {

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables,
// readability-identifier-naming): ファイルスコープのデバイス状態。
K_MUTEX_DEFINE(flash_lock);
const flash_area* flash_area_handle = nullptr;
bool flash_area_opened = false;
// NOLINTEND

}  // namespace

bool flash_store::open_area() {
  if (!flash_area_opened) {
    flash_area_opened = flash_area_open(
                  DT_FIXED_PARTITION_ID(DT_NODELABEL(storage_partition)),
                  &flash_area_handle) == 0;
  }
  return flash_area_opened;
}

bool flash_store::load_latest(uint8_t* out, uint32_t data_len,
                            uint32_t* seq_out) {
  if (!open_area()) {
    return false;
  }
  uint8_t buf[param_store::BLOB_SIZE];
  uint32_t best_seq = 0;
  bool found = false;
  for (uint32_t slot = 0; slot < SLOT_COUNT; ++slot) {
    if (data_len > sizeof(buf) ||
        flash_area_read(flash_area_handle,
                        static_cast<off_t>(slot_offset(slot)), buf,
                        data_len) != 0) {
      continue;
    }
    // ヘッダ + CRC 検証のみ行い、params_ 適用は呼出し側の parse_blob に
    // 委ねる（走査中に複数スロットを適用しないため）。
    if (data_len < param_store::BLOB_SIZE) {
      continue;
    }
    uint32_t magic = 0;
    memcpy(&magic, buf, 4);
    uint16_t ver = 0;
    memcpy(&ver, &buf[4], 2);
    uint32_t seq = 0;
    memcpy(&seq, &buf[6], 4);
    uint32_t crc = 0;
    memcpy(&crc, &buf[param_store::BLOB_HEADER + sizeof(param_set)], 4);
    const bool valid =
        magic == param_store::BLOB_MAGIC &&
        ver == param_store::BLOB_SCHEMA_VER &&
        crc == param_store::crc32(buf, param_store::BLOB_HEADER +
                                          sizeof(param_set));
    if (valid && (!found || seq > best_seq)) {
      memcpy(out, buf, data_len);
      best_seq = seq;
      found = true;
    }
  }
  if (found && seq_out != nullptr) {
    *seq_out = best_seq;
  }
  return found;
}

bool flash_store::write_slot(uint32_t slot, const uint8_t* data,
                             uint32_t data_len) {
  if (!open_area() || slot >= SLOT_COUNT || data_len > SLOT_SIZE) {
    return false;
  }
  const off_t off = static_cast<off_t>(slot_offset(slot));
  k_mutex_lock(&flash_lock, K_FOREVER);
  bool ok = flash_area_erase(flash_area_handle, off, SLOT_SIZE) == 0 &&
            flash_area_write(flash_area_handle, off, data, data_len) == 0;
  if (ok) {
    // 書き込み検証（フラッシュ誤書込みを確実に検出する）。
    uint8_t verify[128];
    uint32_t done = 0;
    while (ok && done < data_len) {
      const uint32_t n =
          (data_len - done < sizeof(verify)) ? data_len - done : sizeof(verify);
      ok = flash_area_read(flash_area_handle, off + static_cast<off_t>(done),
                           verify, n) == 0 &&
           memcmp(verify, &data[done], n) == 0;
      done += n;
    }
  }
  k_mutex_unlock(&flash_lock);
  return ok;
}

bool flash_store::erase_all() {
  if (!open_area()) {
    return false;
  }
  k_mutex_lock(&flash_lock, K_FOREVER);
  bool ok = true;
  for (uint32_t slot = 0; slot < SLOT_COUNT; ++slot) {
    ok = flash_area_erase(flash_area_handle,
                          static_cast<off_t>(slot_offset(slot)),
                          SLOT_SIZE) == 0 &&
         ok;
  }
  k_mutex_unlock(&flash_lock);
  return ok;
}

}  // namespace bb
