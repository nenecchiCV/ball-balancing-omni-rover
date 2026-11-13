# encoder_counter

TIM エンコーダモードの生カウント差分を返す。`encoder_counter` クラス（3 輪分の複数インスタンスが必要なためインスタンス化可能）。devicetree の qdec ノードがタイマをエンコーダモードで初期化するため、本クラスはカウンタ値の読取りと差分計算のみを行う。

## 公開型

```cpp
struct config { TIM_TypeDef* timer; };
```

## 公開関数

- `void init(const config&)` — 対象タイマのレジスタアドレスを保持し、現在の `CNT` を基準値として取り込む。
- `int32_t delta()` — 前回呼び出しからのカウント差分を返す。ARR=65535 の自由周回（`counts_per_revolution=2^n`）のため `int16_t` キャスト差分でラップアラウンドを吸収し、累積 `count_` へ加算する。
- `int64_t count()` — 累積カウント。

## 内部状態

`config_`（タイマアドレス）、`prev_raw_`（前回 CNT）、`count_`（累積値）。

## MATLAB との対応

なし（ハード依存部）。`wheel_encoder` の計測基盤。qdec ドライバは生カウントを公開しないため LL レジスタ読取りとする。
