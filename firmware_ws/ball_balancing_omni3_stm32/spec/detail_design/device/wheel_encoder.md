# wheel_encoder

ホイール用エンコーダ。`wheel_encoder` クラス（3 輪分の複数インスタンスが必要なためインスタンス化可能）。`encoder_counter` の生カウント差分をモデルカウント基準の変位・速度へ換算する。

## 公開型

```cpp
struct config {
    encoder_counter* counter;
    float ticks_per_model_count = 4.0F;  // x4 デコード係数
    float counts_per_rev = 341.2F;        // モデルカウント基準（実機確定）
};
struct sample {
    float displacement;  // rad
    float rate;          // rad/s（呼出窓の後退差分）
};
```

## 公開関数

- `void init(const config&)` — 設定を保持する。
- `sample update(float dt)` — `counter->delta()` を取得し、`ticks_per_model_count` で生カウント→モデルカウントへ換算、`2π/counts_per_rev` で変位 rad、`変位/dt` で速度を返す。

## MATLAB との対応

エンコーダカウントのモデル単位換算。`ticks_per_model_count` / `counts_per_rev` は `params.encoder` から注入する（要実機確認値）。
