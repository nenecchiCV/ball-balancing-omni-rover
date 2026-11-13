# imu_sample

IMU 1 サンプル分のデータ構造。`imu_sample` 構造体。

device 層の IMU ドライバが生成し、feature 層（`state_estimator`）が消費する。Zephyr・ドライバ実装に依存しないため、feature 層はハードウェアなしでホストテスト可能となる。

## 公開型

```cpp
struct imu_sample {
    vec3 specific_force;  // 比力 [m/s^2]（体軸）
    vec3 gyro;            // 角速度 [rad/s]（体軸）
    bool valid = false;   // 読取成功フラグ
};
```

## MATLAB との対応

`ballbotEstimatorStep` の `imu` 入力に相当。体軸への写像は `device/lsm6dso` が `axis_map` で適用する。
