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
};
```

## 公開関数

`const param_set& get_params()` — 全定数を集約した `param_set` への参照を返す（関数ローカル static、`-fno-threadsafe-statics` によりガードなし。`main` 単一スレッド内でのみ使用）。

## 内部状態

なし。値はコンパイル時確定。

## MATLAB との対応

`ballbotRev1Parameters` の数値を `dump_fw_params.m` の JSON 出力から転記する。複素数を含む設計用中間構造（`torqueLqi.poles` 等）は移植対象外とし、実行時に必要な行列・ゲインのみを保持する。`isfield` による有効/無効は `bool` フィールド（`ekf.enabled` 等）で表現する。
