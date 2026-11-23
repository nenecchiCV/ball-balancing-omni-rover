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

- `static void init()` — 積分値・オフセットをゼロ初期化する。
- `static void update(float vx_w, float vy_w, float dt)` — 生積分位置 `raw_x/raw_y` を前進 Euler で更新する。
- `static pose raw()` — オフセット適用前の積分位置。
- `static pose get()` — `raw - origin` の現在ワールド位置。
- `static float yaw_of(float raw_yaw)` — `raw_yaw - yaw_offset` の現在ワールド yaw。yaw 自体は推定器の euler を入力とし、ここでは原点オフセットのみ適用する。
- `static void reset_origin(float raw_x, float raw_y, float raw_yaw)` — 現在の生位置・生 yaw を原点として `origin`/`yaw_offset` に記録する（`RESET_ODOMETRY` 対応）。
- `static void set_pose(float x, float y, float yaw, float raw_x, float raw_y, float raw_yaw)` — `origin`/`yaw_offset` を、現在の `get()`/`yaw_of` が指定値 `(x, y, yaw)` を返すよう設定する（`SET_POSE` 対応。`reset_origin` は `set_pose(0,0,0)` 相当）。

## 内部状態

- `raw_x_`, `raw_y_` — `velocity_world2` の積分値（オフセット前）。
- `origin_x_`, `origin_y_`, `yaw_offset_` — `reset_origin` で記録した原点。

## 備考

- 精度は推定器の `velocity_world2` 品質に従う。長時間ドリフトは許容する（用途は短時間の移動制御）。
- `update` は外側タスク（200 Hz）で推定器出力をそのまま渡す。`reset_origin` はリンクスレッドから呼ばれ得るため、呼出し側が `irq_lock` 等で外側タスクと排他する。
