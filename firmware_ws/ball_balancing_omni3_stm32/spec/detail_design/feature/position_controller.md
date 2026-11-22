# position_controller

ワールド座標の目標位置 (x, y) への位置外ループ。位置誤差からワールド速度指令を生成し、`balance_controller` の指令経路へ渡す（FR-10）。純粋計算で、ターゲットの書換えはリンクスレッドから行われ得るため設定面は `irq_lock` で保護する。

## 公開型

```cpp
class position_controller {  // static class
 public:
  struct config { const param_set* params; };
  struct target { float x; float y; float speed_limit; bool active; };
  struct output { vec3 command{}; bool active; bool arrived; };
};
```

## 公開関数

- `static void init(const config& cfg)` — パラメータ保持と状態リセット。
- `static void set_target(float x, float y, float speed_limit)` — 目標設定（`irq_lock` 内でスロット書換）。`speed_limit` は `min(speed_limit, controller.max_speed)` でクランプし `active=true` にする。
- `static void clear_target()` — 目標を解除する（`active=false`）。
- `static void hold_at(float x, float y)` — `STOP` 対応: 目標を指定位置へ置き換える（`active` は維持）。
- `static output update(float x, float y, float dt)` — 現在位置に対する指令を返す。目標未設定または到達時は零指令。

## 制御則

- `e = target - pose`。`dist = |e|`。
- `dist <= position.arrive_radius` で到達とみなし零指令（`arrived=true`、ホールドのため `active` は維持）。
- 希望速度 `v = position.kp * e` をベクトルノルムで `min(target.speed_limit, controller.max_speed)` に飽和する。
- 加速度制限: `command` の変化量を `position.accel_limit * dt`（ベクトルノルム）で slew する。
- `yaw_rate` 指令は常に 0（目標 yaw は初版で扱わない、FR-10）。

## 内部状態

- `target_`（目標スロット、`irq_lock` 保護）。
- `prev_command_`（slew 用の前回指令、`init` 後または目標更新直後はゼロから再開）。

## 備考

- パラメータは `param_set::position`（`kp=1.0`, `accel_limit=0.5`, `arrive_radius=0.03`）。
- `update` は外側タスクから `link` モードが位置制御のとき呼ばれる。目標スロットの読取りは `irq_lock` で整合コピーしてから計算する。
