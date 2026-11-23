# control_params

`ballbotRev1Parameters()` が生成するパラメータ構造 `p` のファームウェア用スナップショット。全て `constexpr`/集約初期化可能な `param_set` 構造体群として提供する。

## 公開型

```cpp
struct param_set {
    struct physical { ... };      // gravity, ball_radius, ...
    struct motor { ... };         // torque_constant, ...
    struct driver { ... };        // supply_voltage, ...
    struct encoder { ... };       // counts_per_rev, ...
    struct rover { ... };         // mass, ...
    struct geom { ... };          // mat33 行列群
    struct ekf { ... };           // EKF 定数
    struct estimator { ... };     // est.* の全定数
    struct torque_drive { ... };  // td.* の全定数（yaw_gate 含む）
    struct lqi { ... };           // lqi.* 定数
    struct torque_lqi { ... };    // state_gain 等
    struct controller { ... };    // ctrl.* 定数
    struct command { ... };       // cmd.* 全定数
    struct position { ... };      // 位置制御定数（kp, ki, output_max, hold_gain_scale）
    struct link {                 // リンク・動作管理定数
        uint32_t watchdog_ms;      // SET_VELOCITY 失効（500 ms）
        uint32_t idle_ms;          // UART アイドル失効（1000 ms）
        uint32_t event_batch_ms;   // イベントバッチ周期（既定 100 ms, FR-14）
        uint32_t calib_timeout_ms; // CALIBRATE_IMU 完了期限（既定 5000 ms, FR-13）
    };
    struct drive {                // 駆動出力調整（FR-16）
        float deadzone_comp_pos[3]; // 車輪ごとの不感帯補償デューティ 正転用（既定 0.08）
        float deadzone_comp_neg[3]; // 同上 逆転用（既定 0.08）
    };
    struct imu {                  // IMU 特性（FR-15 永続化対象）
        float gyro_bias_z;         // ジャイロ z バイアス（CALIBRATE_IMU で学習）
        float ekf_bias_x;          // EKF 推定 x 軸バイアス
        float ekf_bias_y;          // EKF 推定 y 軸バイアス
    };
    struct tune {                 // 同定セッション定数（FR-17）
        float wake_rate;           // 輪速立上り判定閾値 [rad/s]（既定 2.0）
    };
};
```

## 公開関数

`const param_set& get_params()` — **動作値**への参照を返す。rev C 以降は `param_set` 自体はコンパイル時既定値（`constexpr` 互換の集約初期化）のままだが、実体は `common/param_store` が持つ可変コピーとなり、`get_params()` は `param_store::params()` を返す薄いラッパとする。起動時は「既定値 → `param_store` へ展開 → フラッシュ保存値で上書き」で初期化される（FR-15）。調整対象フィールドは `SET_PARAM`/`GET_PARAMS` の key テーブルで公開され、未公開フィールドはフラッシュ保存のみで link 経由の変更は不可。

## 内部状態

`param_set` 既定値はコンパイル時確定。動作値（可変）は `param_store` の内部状態。

## MATLAB との対応

`ballbotRev1Parameters` の数値を `dump_fw_params.m` の JSON 出力から転記する。複素数を含む設計用中間構造（`torqueLqi.poles` 等）は移植対象外とし、実行時に必要な行列・ゲインのみを保持する。`isfield` による有効/無効は `bool` フィールド（`ekf.enabled` 等）で表現する。
