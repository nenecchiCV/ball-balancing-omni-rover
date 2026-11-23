# param_store

動作パラメータ（`param_set` の RAM 上の可変コピー）の保持・既定値初期化・key 単位 get/set・フラッシュ保存 blob の符号化/復号を担う。フラッシュ自体の読み書きは `peripheral/flash_store` が担い、本コンポーネントは純粋ロジック（ホストテスト対象）とする。

## 公開型・関数

```cpp
class param_store {  // static class
 public:
  struct blob {            // 保存フォーマット（リトルエンディアン）
    // magic u32 | schema_ver u16 | seq u32 | param_set | crc32 u32
  };

  static void init_defaults();            // 動作値を constexpr 既定値へ初期化
  static const param_set& params();       // 動作値への参照（get_params() の後継）
  static param_set& mutable_params();

  // key 単位アクセス（調整対象表は robot_link_protocol.md「調整対象パラメータ」）
  static constexpr uint32_t KEY_COUNT = /* エントリ数 */;
  static bool get_key(uint8_t key, float* value);
  static bool set_key(uint8_t key, float value);   // 範囲外・未定義・読取専用は false
  static uint8_t key_at(uint32_t index);           // GET_PARAMS の昇順列挙用
  static float theoretical_max_speed();            // 派生: 無滑り理論速度上限

  // blob 符号化/復号（フラッシュ非依存）
  static uint32_t blob_size();            // シリアライズ後の総バイト数
  static void build_blob(uint32_t seq, uint8_t* out);   // magic/ver/seq/params/crc32 を書く
  static bool parse_blob(const uint8_t* data, uint32_t len, uint32_t* seq);
                                          // CRC/magic/ver 照合し param_set を適用
  static uint32_t crc32(const uint8_t* data, uint32_t len);
};
```

## 動作仕様

- `init_defaults()` は `param_set` の既定コンストラクタ値を `mutable_params()` へ書き戻す。`robot_application::init` で最初に呼び、その後 `flash_store` の保存値を `parse_blob` で上書きする。
- `set_key` は key → フィールド変換テーブル（`key/field/min/max/read_only` の固定配列、`key_at()` 昇順）に従い、未定義 key・範囲外・`read_only` エントリを `false` で拒否する。反映は `mutable_params()` への書込みのみで、各 feature は既存の `const param_set*` 経由で参照する（`get_params()` は `param_store::params()` を返す実装に変更し、既存の呼び出し側は無変更で動作値を参照する）。
- 派生値: `derive()` は `theoretical_max_speed = physical.wheel_radius × motor.speed_command_limit ÷ sin(physical.contact_latitude)` を内部フィールドへ計算し（`init_defaults`・`parse_blob` 適用時に呼ぶ）、key `0x60`（`controller.max_speed`）のエントリ上限をその値へ更新して現値もクランプする。key `0x61` はこの派生値を返す**読み取り専用**エントリ（`field` は `param_set` 外の内部値を指し、blob には含まれない）。
- `imu` セクションの値は `state_estimator` の学習バイアスと双方向で接続する: 起動時は `params().imu` → `state_estimator::set_bias`、キャリブレーション完了時は `state_estimator::get_bias` → `mutable_params().imu`（`robot_application` が配線）。
- `parse_blob` は `magic`・`schema_ver`・`crc32` 全て妥当な場合のみ適用し `seq` を返す。不整合は `false` で既定値を維持する。
- `build_blob`/`parse_blob`/`crc32`/`set_key`/`get_key` はホストテストで検証する。

## 備考

- `param_set` のレイアウト変更時は `schema_ver` をインクリメントし、旧保存値を無効化する（FR-15）。
- スレッド安全性: `mutable_params()` への書込みはリンクスレッド（`SET_PARAM`/`RESET_PARAMS`）と初期化時のみ。制御スレッドは `params()` を読むのみで、`f32` 単体の書込みはアトミックとみなす。複合フィールドの整合が必要なら `irq_lock` を検討する。
