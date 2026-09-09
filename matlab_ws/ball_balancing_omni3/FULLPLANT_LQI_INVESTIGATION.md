# Full Multibody LQI 再設計調査

## 結論

> **現行構成:** 上部ロッド仕様は `ball_balancing_omni3_multibody_lqi_custom_contact.slx` に正式採用した。調査モデル `ball_balancing_omni3_multibody_lqi_fullplant.slx` は `old/` に退避した。現行LQIはローバー総質量1.085 kg、球中心からの合成重心高約0.201 mを用いる低次モデルで再設計している。3輪custom contact実装前のため、非線形閉ループ安定性は未検証である。

`ball_balancing_omni3_multibody_lqi_fullplant.slx` を基準モデルから派生させ、LQI出力後に速度PIとDCモータを明示的に接続した。さらに、実Full Multibodyプラントの整合した直立Operating Pointとfiniteな解析線形モデルを取得できた。

> **モデル更新注意:** 保存済みOperating Point・線形化・FRFは保持拘束を削除する前に生成した結果である。現在の拘束なしモデルに対しては再取得が必要であり、そのまま制御設計へ使用してはならない。

ただし、Full Multibody由来ゲイン候補は非線形モデルの1秒検証で転倒したため、最終制御則としては未採用である。派生モデルは調査・継続設計用成果物として保存したが、直立安定化のAcceptance Criteriaを満たしたとは扱わない。`ballbotReducedPlantDynamics` は参照用に残し、今回の線形化とゲイン候補のdesign authorityには使用していない。

## 実I/O境界

プラント入力はLQIが出力する3輪速度指令、出力は次の10信号である。

`[v_Bx, v_By, pitch, -roll, q, -p, r, wheelRate1, wheelRate2, wheelRate3]`

この境界の内側には速度PI、MDD3A電圧・電流制限、FIT0521 DCモータ、3輪Spatial Contact Force、球―床接触、ball/body Multibodyを含む。派生モデルの `SpeedPiAndDcMotor` は `ballbotSpeedPiMotorStep` を呼び、PI積分状態はMemoryブロックで保持する。

## Operating Point

球―床penalty contactを無支持の接触境界から開始しないよう、実モデルの鉛直加速度を探索して静的侵入量を `1.43e-4 m` とした。`t = 0.005 s` の閉ループsnapshotから、離散コントローラ状態を既知かつ非定常として物理Multibody状態だけをtrimした。

`findop` の最大制約違反は `1.31484262e-7` で、直立点では球―床と3輪接触の全4接触が成立した。Operating Pointとレポートは `fullplant_operating_point.mat` に保存した。

## 解析線形化

同一I/O境界で、サンプル時間 `0.005 s` の10出力・3入力・38状態モデルを得た。A/B/C/Dはfinite、Dは0、可制御ランクと可観測ランクはいずれも6、最大極絶対値は `2.69199` であった。高周波接触モードと低周波姿勢・速度モードが混在するため、modal decompositionで速度PIと姿勢に対応する9状態を抽出した。9状態近似の0.1～50 rad/s相対誤差は約0.36～0.40であり、完全な低次近似ではない。

解析結果は `fullplant_linearization.mat` に保存した。3輪対称形状によりX/Y直接結合は小さいが、接触モードを含む全帯域では完全な独立2軸として扱えないため、3入力10出力MIMOのまま評価した。

## FRFクロスチェック

初期の開ループPRBS試験は不安定平衡点から離脱し、解析線形化と一致しなかった。これは不安定機体では閉ループ安定化後に同定するという既存研究の手順とも整合する。保存済み `fullplant_identification_evidence.mat` は、この不成立結果を成功として扱わないための証拠である。

今回のゲイン候補では非線形閉ループ安定化を達成できなかったので、有効な閉ループFRFを新たな合格根拠として採用していない。次段階は、安定化済み制御器の入力に小振幅multisineを重畳し、同じ閉ループI/Oの `linearize` と複数振幅 `frestimate` を比較することである。

## LQI候補と非線形検証

38状態モデルをmodal decompositionし、速度PIの3モードと低周波姿勢モードを含む9状態モデルに対して速度誤差積分2状態を付加し、離散LQI候補を設計した。状態フィードバックを10個の実feedback信号へ射影し、非線形Full Multibodyで符号と姿勢・角速度比を探索した。

0.2度の単軸初期傾斜では0.3秒時点の姿勢を改善する候補が得られたが、1秒では単軸・他軸・複合傾斜の全ケースが転倒し、球―床接触も喪失した。したがって候補ゲインを `fullplant_lqi_design.mat` として採用保存せず、派生モデルは既定ゲインで起動する。

## 座標・符号監査

単位入力に対する解析線形モデルの初期応答を3輪幾何へ写像すると、平面X速度指令はX速度を正方向へ、pitchを負方向へ加速し、平面Y速度指令はY速度を正方向へ、`-roll` を負方向へ加速した。X/Y相互成分は数値丸め程度であり、`[pitch, -roll, q, -p]` と輪配置行列の組合せは局所的に整合している。

一方、Multibody初期姿勢だけに傾斜を設定し、推定器Quaternionを常に直立で初期化していた不整合を確認した。派生モデルの `EstimatorStateMemory` は `ballbotEstimatorInitialState(ballbotParams)` を使用するよう修正し、実姿勢と推定器初期姿勢を一致させた。基準モデルからの引数なし呼出しは従来どおり直立を返す。

球と車体中心を結んでいた `ContactRetentionJoint` と `ContactRetentionOffset` は派生モデルから削除した。現在の球―車体間の支持は3輪penalty contactだけである。

保持拘束を外した後、傾斜時の車体位置を直立時のままにしていた初期assembly不整合も修正した。`ballbotRoverInitialPosition` により、車体姿勢だけでなく車体中心も球中心回りに回転させる。派生モデル用wheel preloadは0.05 mmから0.20 mmへ変更した。0.50 mm以上は過大なpenalty反力を生じたため採用していない。

無制御では17.5度を6方位で試験し、全3輪contactを維持した。制御有効では17.0度まで維持し、17.25度で上側1輪の離脱が現れたため、Recovery開始角を18度から17度へ変更した。これにより通常制御状態からRecoveryへ入るまでの試験範囲では3輪contactを維持する。

## 既存研究から反映した方針

- MiaPUREの実機構成に合わせ、外側の平面LQR/LQIと内側のモータ速度PIをcascade化した。
- 不安定平衡点の同定は、まず閉ループで安定化してから実施する方針とした。
- 3D ballbotのパラメータ感度、とくに質量・接触モデルずれを無視せず、非線形Full Multibody検証を採用判定にした。
- 2D独立軸近似はsanity checkに限定し、3輪幾何と接触を含むMIMO応答を設計対象とした。

## 高重心ペイロード

車体重心を高くするため、派生モデルの車体フレーム上面へ `PayloadRod500g` を剛体接続した。ロッドは質量0.50 kg、長さ0.30 m、断面20 mm角のBrick Solidで、下端を車体上面へ合わせている。追加後の車体系総質量は1.085 kgである。既存質量が車体原点へ集中している近似では、合成重心は車体原点から約76 mm上昇し、球中心から約201 mm上方となる。

追加質量を球―床の静的侵入量計算にも含めた。無制御直立0.1秒の確認では全4接触を維持し、球鉛直振幅は約0.0122 mm、車体鉛直振幅は約0.0402 mmで、シミュレーションはfiniteだった。保持拘束は再導入していない。

## 再現ファイル

- `ballbotFullPlantParameters.m`: penalty contact初期化と派生モデル用パラメータ
- `fullplant_operating_point.mat`: trim済みOperating Point、仕様、残差
- `fullplant_linearization.mat`: 同一I/Oの38状態解析線形化
- `review_fullplant_design.m`: 保存結果のfinite性、極、ランク、trim残差を表示
- `tests/fullplantDesignArtifactsTest.m`: 現行モデル、正式上部ロッドパラメータ、保存証拠の回帰検査

## 参考文献

- MiaPURE, “Balancing Control of a Ball-Riding Robot” (2023): https://arxiv.org/abs/2304.02887
- “Closed-loop identification and control of a ballbot” (2024): https://arxiv.org/abs/2404.14845
- MathWorks, Spatial Contact Force: https://www.mathworks.com/help/sm/ref/spatialcontactforce.html
- “LQR control of a 3D ballbot and sensitivity to mass uncertainty”: https://onlinelibrary.wiley.com/doi/full/10.1002/msd2.12133
- “Development of Ballbot LQR Control System”: https://pure.seoultech.ac.kr/en/publications/development-of-ballbot-lqr-control-system/
