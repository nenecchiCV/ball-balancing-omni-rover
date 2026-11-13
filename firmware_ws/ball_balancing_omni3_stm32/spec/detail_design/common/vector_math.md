# vector_math

固定長ベクトル・行列・quaternion の純粋数学ユーティリティ。依存なし、全て `inline`/`constexpr` 可能なヘッダのみの実装とする。STL は使わず、固定長配列は ETL（`etl::array`）を使用する。

## 公開型

```cpp
using vec2  = etl::array<float, 2>;
using vec3  = etl::array<float, 3>;
using vec4  = etl::array<float, 4>;
using vec10 = etl::array<float, 10>;
using mat33 = etl::array<etl::array<float, 3>, 3>;
using mat55 = etl::array<etl::array<float, 5>, 5>;
```

## 公開関数（代表）

- `dot`/`cross`/`norm`/`normalize`/`max_abs`（vec2/vec3）
- `mul`（mat33×vec3）
- `inverse3`
- quaternion 積・回転行列変換・オイラー変換・積分（実装は `vector_math.hpp` を参照）

## 内部状態

なし（純粋関数のみ）。

## MATLAB との対応

`ballbotEstimatorStep` 内の `skewMatrix`・`integrateQuaternion`・`quaternionToRotation`・`rotationToEuler`・`quaternionMultiply` をそのまま写像。`pinv(W_map)` は 3x3 逆行列で実現（本系の W_map はフルランク）。
