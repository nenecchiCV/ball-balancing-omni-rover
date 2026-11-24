# encoder_counter

TIM エンコーダモードの生カウント差分を返す。`encoder_counter` クラス（3 輪分の複数インスタンスが必要なためインスタンス化可能）。devicetree の qdec ノードがタイマをエンコーダモードで初期化するため、本クラスはカウンタ値の読取りと差分計算のみを行う。

## 公開型

```cpp
struct config { TIM_TypeDef* timer; };
```

## 公開関数

- `void init(const config&)` — 対象タイマのレジスタアドレスを保持し、現在の `CNT` を基準値として取り込む。
- `int32_t delta()` — 前回呼び出しからのカウント差分を返す。`int16_t` キャスト差分でラップアラウンドを吸収し、累積 `count_` へ加算する。本方式が成り立つにはタイマの折返し境界（ARR+1）が 2^16 の倍数であることが必須で、devicetree の `st,counts-per-revolution = <65536>` により実現する（16bit タイマは ARR=0xFFFF の自由周回、32bit TIM2 は ARR=0xFFFF0000）。それ以外の値では折返し点で差分が誤った大きな値に跳ぶ。
- `int64_t count()` — 累積カウント。

## 内部状態

`config_`（タイマアドレス）、`prev_raw_`（前回 CNT）、`count_`（累積値）。

## MATLAB との対応

なし（ハード依存部）。`wheel_encoder` の計測基盤。qdec ドライバは生カウントを公開しないため LL レジスタ読取りとする。
