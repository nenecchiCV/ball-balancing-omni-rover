# flash_store

`storage_partition`（`flash0` の `partition@7f000`、4 KB）への blob 読出・消去・書込を担う。Zephyr flash ドライバ（`flash_area` API）に閉じ込め、ペリフェラル層として他のフラッシュ依存を露出しない。

## 公開型・関数

```cpp
class flash_store {  // static class
 public:
  static constexpr uint32_t SLOT_COUNT = 2;     // 2 スロット交互書き
  static constexpr uint32_t SLOT_SIZE = 2048;   // G474 ページ 2 KB × 1 面/スロット

  static bool init();                            // flash_area_open（失敗で false）
  static bool read(uint8_t slot, uint8_t* out, uint32_t len);      // slot 先頭から len B
  static bool write(uint8_t slot, const uint8_t* data, uint32_t len); // 消去+書込（ms 級ブロック）
  static bool erase(uint8_t slot);               // slot 消去のみ
};
```

## 動作仕様

- 対象は `FIXED_PARTITION(storage_partition)`。`init` で `flash_area_open` し、以後 `flash_area_read`/`flash_area_erase`/`flash_area_write` を使う。スロット `i` のオフセットは `i * SLOT_SIZE`（2 面×2 KB で計 4 KB）。
- `write` は対象スロットを消去してから `len` B を書込み、書込み完了までブロックする（消去は ms 級。呼出し側が待機中限定であることは呼出し側責務＝FR-15）。
- パーティション不在・open/erase/write 失敗は `false` を返し、呼出し側で既定値フォールバックまたは ACK 失敗とする。
- Kconfig: `CONFIG_FLASH=y` + `CONFIG_FLASH_MAP=y` + `CONFIG_FLASH_PAGE_LAYOUT=y` を `prj.conf` に追加する。

## 備考

- 書込み途中の電源断は「当該スロットの CRC 不整合」で検出されるため、前回スロットが生存していれば復旧できる（2 スロット方式の理由）。
- G474RE のシングルバンク構成ではページ 2 KB。`SLOT_SIZE` はボードのページサイズに合わせる。
