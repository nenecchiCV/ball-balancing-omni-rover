# wheel_speed_meter

5 ms 窓の後退差分で 3 車輪の輪速を計測し、1 kHz タスクには保持値を供給する。`wheel_speed_meter` static class。

## 公開型

```cpp
struct config { wheel_encoder* enc[3]; float window = 0.005F; };
```

## 公開関数

- `static void init(const config&)` — エンコーダ参照と計測窓を保持し、保持輪速をリセットする。
- `static vec3 update()` — 5 ms 毎に呼ばれ、各 `enc[i]->update(window).rate` で `rate_` を更新して返す。
- `vec3 rate() const` — 直近の `rate_` を返す（1 kHz タスク用の保持値）。

## 内部状態

`rate_`（保持輪速）。

## MATLAB との対応

`ballbotWheelRateFromDisplacement`（5 ms 窓）相当。内側タスクが参照する `wheel_rate` はこの保持値。
