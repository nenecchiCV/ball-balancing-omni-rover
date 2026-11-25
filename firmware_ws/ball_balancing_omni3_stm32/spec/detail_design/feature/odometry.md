# odometry

ワールド速度の積分による 2 次元位置推定と、ワールド原点（起動点）オフセット管理。純粋計算のみで I/O を持たない（FR-10, FR-11）。制御スレッド（外側タスク）からのみ呼ばれる想定で、排他制御は呼出し側に委ねる。

## 公開型

```cpp
class odometry {  // static class
 public:
  struct pose { float x; float y; };
};
```

## 公開関数

- `static void init()` — 積分値・オフセット・慣性チャネルをゼロ初期化する。
- `static void update(float vx_w, float vy_w, float dt)` — 生積分位置 `raw_x/raw_y` を前進 Euler で更新する（動作中用）。待機中慣性チャネルの速度はここで無効化し、次回待機突入時に張り残さない（加速度バイアスは学習済み値として保持する）。
- `static void update_inertial(float ax_w, float ay_w, bool still, float dt)` — 待機中専用の IMU 慣性オドメトリ（FR-11、検証用途）。ワールド水平加速度をバイアス補正後に 2 重積分して `raw_x/raw_y` を進める。`still`（推定器の静止候補）が `K_ZUPT_HOLD_TIME=0.3 s` 継続すると速度をゼロへ張り付け（ZUPT）、かつ確定静止中は水平加速度残差を `K_ACCEL_BIAS_TIME_CONSTANT=2.0 s` 時定数の EMA でバイアス学習して非静止中の積分入力から差し引く。等速運動は検知できず、位置精度は搬送時間に比例して劣化する表示用参考値とする。動作開始時は `reapply_declared` で宣言位置へ再アンカーされるため誤差は動作へ持ち越されない。
- `static pose inertial_velocity()` — 待機中慣性チャネルの現在速度 `{vx, vy}`（EVT_POSE 表示用）。
- `static pose raw()` — オフセット適用前の積分位置。
- `static pose get()` — `raw - origin` の現在ワールド位置。
- `static float yaw_of(float raw_yaw)` — `raw_yaw - yaw_offset` の現在ワールド yaw。yaw 自体は推定器の euler を入力とし、ここでは原点オフセットのみ適用する。
- `static void reset_origin(float raw_x, float raw_y, float raw_yaw)` — 現在の生位置・生 yaw を原点として `origin`/`yaw_offset` に記録する（`RESET_ODOMETRY` 対応）。宣言位置は原点 `(0,0,0)` とする。
- `static void set_pose(float x, float y, float yaw, float raw_x, float raw_y, float raw_yaw)` — `origin`/`yaw_offset` を、現在の `get()`/`yaw_of` が指定値 `(x, y, yaw)` を返すよう設定する（`SET_POSE` 対応。`reset_origin` は `set_pose(0,0,0)` 相当）。指定値は宣言位置 `decl_*` として記憶する。
- `static void reapply_declared(float raw_x, float raw_y, float raw_yaw)` — 最後に宣言した位置・向きを現在の生位置・生 yaw へ再適用する（`START_OPERATION` 受理時、FR-12）。待機中に蓄積した生積分ドリフト（手で車体を動かした等）を動作開始へ持ち込まないための再アンカーで、`set_pose(decl_x_, decl_y_, decl_yaw_, ...)` と等価。

## 内部状態

- `raw_x_`, `raw_y_` — `velocity_world2`（動作中）または慣性速度（待機中）の積分値（オフセット前）。
- `origin_x_`, `origin_y_`, `yaw_offset_` — `reset_origin`/`set_pose` で記録した原点。
- `decl_x_`, `decl_y_`, `decl_yaw_` — 最後に宣言した位置・向き（`reapply_declared` の再適用先）。`reset_origin` では原点、初期値も原点。
- `inertial_vx_`, `inertial_vy_` — 待機中慣性チャネル速度（ZUPT 対象）。
- `accel_bias_x_`, `accel_bias_y_` — 確定静止中に学習する水平加速度バイアス。
- `still_time_` — 静止候補の連続時間（ZUPT 確定判定用）。

## 備考

- 精度は動作中は推定器の `velocity_world2` 品質に従う。待機中は慣性チャネルが検証用の位置変位を示すが、IMU 単独では等速運動を検知できないため精度は参考値に留まる。いずれにせよ動作開始時に宣言位置を再適用してリセットする（FR-12）。
- `update`（動作中）と `update_inertial`（待機中、`imu_sample.valid` のとき）は外側タスク（200 Hz）から推定器出力をそのまま渡す。`reset_origin`/`set_pose` はリンクスレッドから、`reapply_declared` は外側タスクから呼ばれるため、呼出し側が `irq_lock` 等で排他する。
