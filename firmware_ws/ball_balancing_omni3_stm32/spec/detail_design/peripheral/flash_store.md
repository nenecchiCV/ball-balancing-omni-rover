# flash_store

`storage_partition`（`flash0` の `partition@7f000`、4 KB）への blob 読出・消去・書込を担う。Zephyr flash ドライバ（`flash_area` API）に閉じ込め、ペリフェラル層として他のフラッシュ依存を露出しない。

## 公開型・関数

```cpp
class flash_store {  // static class
 public:
  static constexpr uint32_t SLOT_COUNT = 2;     // 2 スロット交互書き
  static constexpr uint32_t SLOT_SIZE = 2048;   // G474 ページ 2 KB × 1 面/スロット

  // 2 スロットを走査し、seq 最大の妥当 blob を out（data_len 以上）へ読み込む
  static bool load_latest(uint8_t* out, uint32_t data_len, uint32_t* seq_out);
  // slot を 1 ページ消去して data を書き込む（検証込み、100 ms 級ブロック）
  static bool write_slot(uint32_t slot, const uint8_t* data, uint32_t data_len);
  static bool erase_all();                       // 両スロットを消去
};
```

## 動作仕様

- 対象は `FIXED_PARTITION(storage_partition)`。初回アクセス時に `open_area` で `flash_area_open` し、以後 `flash_area_read`/`flash_area_erase`/`flash_area_write` を使う。スロット `i` のオフセットは `i * SLOT_SIZE`（2 面×2 KB で計 4 KB）。
- `load_latest` は両スロットの先頭ヘッダを読んで seq を比較し、最大 seq のスロットを `data_len` B 読み込む（blob 自体の CRC 検証は呼出し側の `param_store::parse_blob` が担う）。妥当スロットが 1 つもなければ `false`。
- `write_slot` は対象スロットを消去してから `data_len` B を書込み、書き戻し検証までブロックする（消去は ms 級。呼出し側が待機中限定であることは呼出し側責務＝FR-15）。
- パーティション不在・open/erase/write 失敗は `false` を返し、呼出し側で既定値フォールバックまたは ACK 失敗とする。
- Kconfig: `CONFIG_FLASH=y` + `CONFIG_FLASH_MAP=y` + `CONFIG_FLASH_PAGE_LAYOUT=y` を `prj.conf` に追加する。

## 備考

- 書込み途中の電源断は「当該スロットの CRC 不整合」で検出されるため、前回スロットが生存していれば復旧できる（2 スロット方式の理由）。
- G474RE のシングルバンク構成ではページ 2 KB。`SLOT_SIZE` はボードのページサイズに合わせる。
