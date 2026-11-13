# lsm6dso

LSM6DSO IMU（Zephyr センサドライバ経由）。`lsm6dso` static class（IMU は 1 個）。デバイス設定（ODR 833 Hz、±4 g、±2000 dps）は devicetree で確定済み。本クラスは軸写像のみを担当する。

## 公開型

```cpp
struct config {
    const struct device* dev;
    mat33 axis_map;  // 既定は単位行列。実装向きで確定
};
using sample = imu_sample;
```

## 公開関数

- `static bool init(const config&)` — `device_is_ready` でドライバを検査し、軸写像行列を保持する。
- `static sample read()` — `sensor_sample_fetch` + `sensor_channel_get(ACCEL_XYZ/GYRO_XYZ)` で取得し、`axis_map_` で軸写像した `imu_sample` を返す。取得失敗時は `valid=false`。

## 内部状態

`dev_`（デバイスハンドル）、`axis_map_`（軸写像行列）。

## MATLAB との対応

`imuData` の読取り。生値に対し実装向きの軸変換 `axis_map_` を適用する点のみ追加。
