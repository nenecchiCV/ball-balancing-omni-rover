# command_profile

運転指令 `[vx_W; vy_W; yawRate]` を生成する。`command_profile` static class。

## 公開型

```cpp
struct config { const param_set* params; };
struct output { vec3 command; bool enable; };
```

## 公開関数

- `static void init(const config&)` — パラメータ参照を保持し、内部時計と指令状態を初期化する。
- `static output update(float now_sec)` — 時刻に応じて指令を返す。`cmd.cycles_enabled` 時はサイクルプロファイル（走行→停止→ヨーパルス）を `cycle_count` 回繰返し、そうでなければ `cmd.velocity_world`/`yaw_rate` の固定指令を返す。速度は加速度/減速度リミットで台形整形される。

## 内部状態

`prev_velocity_`（台形整形の前回指令速度）、`prev_time_`。

## MATLAB との対応・適応事項

`ballbotParameters` の `command` 構造に対応し、サイクルプロファイル生成ロジックを移植する。`now_sec < cycle_start` では `elapsed` が 0 にクランプされるため「サイクル先頭」として扱われる挙動は MATLAB 側と同じである（指令自体は台形整形により 0 から立ち上がる）。
