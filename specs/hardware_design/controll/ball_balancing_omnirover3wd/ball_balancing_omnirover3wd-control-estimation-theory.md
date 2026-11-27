# 玉乗り3WDオムニローバー制御・状態推定理論

## 状態

| 項目 | 値 |
|---|---|
| ステータス | 実装一致版（rev2 対応） |
| 最終更新日 | 2026-10-22 |
| 実装 | `matlab_ws/ball_balancing_omni3/*.m` |
| 対象モデル | `ball_balancing_omni3_multibody_lqi_rev2.slx` |

## 1. 座標・姿勢

$$
{}^Wv_d=\begin{bmatrix}v_{x,d}^W\\v_{y,d}^W\end{bmatrix},\qquad
{}^Bv_d=R_z(\hat\psi)^T{}^Wv_d
$$

$$
R_z(\psi)=
\begin{bmatrix}
\cos\psi&-\sin\psi\\
\sin\psi&\cos\psi
\end{bmatrix}
$$

| 姿勢角 | 正方向 | 機体上端の初動 |
|---|---|---|
| ロール $\phi$ | $+X_B$右手 | $-Y_B$ |
| ピッチ $\theta$ | $+Y_B$右手 | $+X_B$ |
| ヨー $\psi$ | $+Z_B$右手 | 機首が$+Y_W$側へ回転 |

## 2. 3輪球面接触幾何

$$
\beta_i\in\{0,2\pi/3,4\pi/3\},\qquad\lambda=33\ \mathrm{deg}\;(0.576\ \mathrm{rad})
$$

$$
n_i^B=
\begin{bmatrix}
\cos\lambda\cos\beta_i\\
\cos\lambda\sin\beta_i\\
\sin\lambda
\end{bmatrix},\quad
t_i^B=
\begin{bmatrix}
-\sin\beta_i\\
\cos\beta_i\\
0
\end{bmatrix},\quad
a_i^B=n_i^B\times t_i^B
$$

| ベクトル | 物理意味 | 直交関係 |
|---|---|---|
| $n_i$ | ボール中心から接触点への法線 | $n_i^Tt_i=n_i^Ta_i=0$ |
| $t_i$ | オムニホイール駆動転動方向 | $t_i^Ta_i=0$ |
| $a_i$ | ホイール車軸・ローラー自由方向 | $a_i=n_i\times t_i$ |

ホイール軸トルク$\tau_i$に対応する駆動力$F_i=\tau_i/R_w$とボール中心回りトルクは、

$$
\tau_{b,i}^B=R_bn_i^B\times(F_it_i^B)
=\frac{R_b}{R_w}a_i^B\tau_i
$$

$$
\tau_b^B=A_\tau\tau_w,\qquad
A_\tau=\frac{R_b}{R_w}
\begin{bmatrix}a_1^B&a_2^B&a_3^B\end{bmatrix}
$$

上式は3輪トルクがボールへ及ぼす一般化トルクの物理関係である。現行の制御器はトルク入力LQIで姿勢トルクを生成し、推進チャネルは輪速空間で目標を生成する。輪速目標への写像として同じ幾何から

$$
\omega_{w,d}=W_\omega
\begin{bmatrix}u_{planar}\\ r_d\end{bmatrix},\qquad
W_\omega=
\begin{bmatrix}
-\dfrac{\sin\lambda}{R_w}G_{r,xy}^T & \dfrac{R_c}{R_w}\mathbf{1}_3
\end{bmatrix}
$$

を用いる。$G_{r,xy}$は3輪の転動方向ベクトル$t_i^B$のXY成分を並べた$2\times3$行列（`p.wheel.geometry.rollingBody(1:2,:)`）、$R_c$はシャーシ半径である。逆方向には$W_\omega(:,1{:}2)$の疑似逆行列で機体平面速度を輪速から再構成する。

対応実装: `ballbotWheelGeometry.m`（幾何）、`ballbotFullPlantParameters.m`（$W_\omega$生成）。

## 3. 接触モデル

### 3.1 法線

$$
F_n=s(d,w)\max(k_nd+c_n\dot d,0)
$$

| 接触 | $k_n$ [N/m] | $c_n$ [N/(m/s)] | $w$ [m] |
|---|---:|---:|---:|
| ホイール–ボール | $1.36\times10^{4}$（ボール膜 $3.0\times10^{4}$ と goBILDA 50A ゴムローラー層 $2.5\times10^{4}$ の直列合成） | 170 | $5.0\times10^{-4}$ |
| ボール–床 | $3.0\times10^{4}$ | 500 | $5.0\times10^{-4}$ |

剛性の不確かさはロバスト性受け入れ試験（[検証計画](ball_balancing_omnirover3wd-test-plan.md)）で $k\in\{1\times10^{4},3\times10^{4},6\times10^{4}\}$ N/m を掃引して担保する。

### 3.2 オムニホイール異方性摩擦

接触座標の相対接線速度$v_t^C$を駆動方向$d^C$とローラー方向$r^C$へ分解する。

$$
v_d=(d^C)^Tv_t^C,\qquad v_r=(r^C)^Tv_t^C
$$

$$
F_t^C=-\mu_dF_n\tanh(v_d/v_c)d^C
-\mu_rF_n\tanh(v_r/v_c)r^C
$$

| パラメーター | 値 |
|---|---:|
| $\mu_s=\mu_d$（駆動方向） | 2.0 |
| $\mu_r$（ローラー方向） | 0.02 |
| $v_c$ | 0.005 m/s |

球–床接触は等方性 $\mu_s=\mu_d=1.2$ とし、加えて転がり抵抗を `ballbotRollingResistance` が印加する（係数 `p.contact.ballGround.rollingFriction`=0.008、臨界速度 0.005 m/s、スピンスクラブ項 `spinScrubArm`=1 mm・`spinScrubCriticalRate`=0.5 rad/s）。転がり抵抗係数の許容帯は 0.005–0.015（フローリング〜短毛絨毯想定）で、帯全域の生存は受け入れ試験で確認する。トラクション推定用の摩擦係数は `tractionEstimateFriction`=1.0（実物摩擦 2.0 に対し保守側・80%ディレート込み）。

対応実装: `ballbotCustomFriction.m`、`ballbotRollingResistance.m`。

## 4. IMU観測式

$$
f^B=R_{WB}^T(a_B^W-g^W),\qquad
g^W=\begin{bmatrix}0&0&-g\end{bmatrix}^T
$$

$$
y_{IMU}=\begin{bmatrix}f^B\\\omega_B^B\end{bmatrix}
$$

対応実装: `ballbotImuFromJoint.m`→`ballbotIdealImu.m`。

## 5. 推定器

### 5.1 状態・出力

$$
x_e=
\begin{bmatrix}
q_{WB}^T&({}^Wv_B)^T&({}^W\omega_K)^T&({}^Bp_{B/K,xy})^T&
\hat b_{g,z}&t_{qual}
\end{bmatrix}^T\in\mathbb{R}^{14}
$$

$$
\hat z=
\begin{bmatrix}
\hat\phi&\hat\theta&\hat\psi&\hat p&\hat q&\hat r&
\hat v_x^W&\hat v_y^W&
({}^W\hat\omega_K)^T&
({}^B\hat p_{B/K,xy})^T
\end{bmatrix}^T
$$

外部推定出力は$\hat z\in\mathbb{R}^{13}$とする。角速度$[\hat p,\hat q,\hat r]^T$は生ジャイロ値ではなく、前回サンプルまでの$\hat b_{g,z}$を差し引いた値とする。モデル内の推定バスは保存済みSelector範囲を維持するため14幅のままとし、末尾要素は未使用パッド（常時0）である。

診断出力は次の3要素を別信号で公開する。

$$
d_b=\begin{bmatrix}\hat b_{g,z,k+1}&\gamma_k&t_{qual,k+1}\end{bmatrix}^T
$$

### 5.2 姿勢更新（レガシー比例補正・フォールバック）

現行既定は §5.2b の誤差状態EKFである。本節のゲート付き比例補正は `p.estimator.ekf.enabled=false` のフォールバック経路として残置する。

$$
u_g^B=R_{WB}^Te_3,\qquad
u_a^B=\frac{f^B}{\|f^B\|}
$$

$$
\bar\omega_{B,k}^B=\omega_{IMU,k}^B-
\begin{bmatrix}0&0&\hat b_{g,z,k}\end{bmatrix}^T
$$

$$
\omega_c^B=\bar\omega_B^B+K_a(u_a^B\times u_g^B)
$$

補正のゲート:

$$
\left|\|f^B\|-g\right|\le0.25g
$$

$$
q_{k+1}=\operatorname{normalize}\left(q_k+\frac{T_s}{2}\Omega(\omega_c^B)q_k\right)
$$

| パラメーター | 値 |
|---|---:|
| $T_s$ | 0.005 s |
| $K_a$ | 2.5 s$^{-1}$ |

### 5.2b 誤差状態EKF（現行既定、`p.estimator.ekf.*`）

現行構成では上記のゲート付き比例補正を**誤差状態EKF**へ置き換えた（`ekf.enabled=true`・`useLegacyCorrection=false`）。誤差状態ベクトルは5状態（姿勢誤差$\delta\theta\in\mathbb{R}^3$、ロール/ピッチ軸ジャイロバイアス誤差$\delta b_{xy}\in\mathbb{R}^2$）で、クォータニオンは名目状態として非誤差側に保持する。ヨー軸ジャイロバイアスは絶対方位参照がなく本EKFでは観測不能なため、従来どおり §5.6 の低運動認定学習に委ねる。

**予測**: バイアス補正後ジャイロでクォータニオンを伝播し、

$$
\Phi_k=I-\Delta t\,[\bar\omega_B^B]_\times,\qquad
P^+=\Phi P\Phi^T+Q_k
$$

で共分散を伝播する。$Q_k$はLSM6DSOのデータシート値（ジャイロノイズ密度 3.8 mdps/√Hz、ゼロレートオフセット ±1 dps）から導出した離散プロセスノイズとする。

**観測更新**: 加速度計比力を重力モデル$h=-R_{WB}^Tg^W$で観測し、感度行列$H=[\,\lfloor\hat f^B\rfloor_\times\ \ 0_{3\times2}\,]$、Joseph形で共分散を更新し、$\delta\theta$を小角クォータニオン注入で名目クォータニオンへ畳み込む。

**運動モデルによる有効性判定**: 本機は動作原理上ほぼ常時加速するため、加速度計比力は重力と実加速度の和であり、無条件の重力補正は姿勢を幻影傾斜$a/g$へ引く（実測: 連続補正で推定が±1.8 radまで振れた）。補正注入を試みた全変種（ボール加速度単独、$+z_{com}\ddot\theta$項、$+\ddot{\delta}_{rel}$項を残差内に含むモデル）が寄生相殺または自己参照ブートストラップ（relPos/速度推定が姿勢推定由来）で閉ループ保持を悪化させたため、運動モデルは残差ではなく**更新可否の判定**に用いる:

$$
a_{pred}={}^W\ddot p_{ball}+R_{WB}\ddot\delta^B_{rel}+z_{com}(\dot\omega^W\times\hat z)
$$

をエンコーダ運動学・相対位置状態・ジャイロ微分から構成し（全て姿勢推定と独立）、$|a_{pred}|$・$|\omega_{rp}|$・車輪スルーレートの3系統が連続0.30 s静寂である場合に限り更新を許可する。これにより停止時と定常巡航時には動作中も重力補正が流れ（比例補正のみの旧構成に対するEKFの利得）、過渡・スロッシュ・制動中はジャイロ単独伝播となる。1更新あたりの傾き注入は0.004 radにクランプし、極端残差（$>1.5\ \mathrm{m/s^2}$）のみ拒否する。

実測: 静止保持 maxTilt 0.0023 rad（旧補正 ~0.011 rad の約1/5）、50 s cycle 完走、0.40 m/s指令が転がり抵抗0.005–0.015全帯で完走。

### 5.3 平面速度

$$
{}^Wa_B=R_{WB}f^B+g^W
$$

$$
{}^Wv_{B,k+1}=\alpha_v{}^Wv_{B,k}+T_s{}^Wa_B,\qquad
\alpha_v=0.9995
$$

平面モデルでは$v_{B,z}=0$を課す。

### 5.4 ボール角速度

エンコーダーの直接観測量は車輪回転変位$\theta_{w,i}$とし、FIT0521の公称分解能341.2 PPRに対応する量子化幅$2\pi/341.2$ radをパラメーター化する。推定器とDCモータ速度包絡線で用いる車輪角速度は5 ms周期の後退差分で算出する。初回サンプルでは前回変位を現在変位で初期化し、$\omega_{w,i}[0]=0$とする。

$$
\omega_{w,i}[k]=\frac{\theta_{w,i}[k]-\theta_{w,i}[k-1]}{T_s},\qquad T_s=0.005\ \mathrm{s}
$$

球–床無すべり近似:

$$
{}^Wv_K=G_R{}^W\omega_K,\qquad G_R=
\begin{bmatrix}
0&R_b&0\\
-R_b&0&0\\
0&0&0
\end{bmatrix}
$$

ホイール$i$の駆動方向無すべり残差:

$$
\epsilon_i=(t_i^W)^T
\left[
{}^Wv_B+{}^W\omega_B\times{}^Wp_{F_i/B}
-{}^Wv_K-{}^W\omega_K\times(R_bn_i^W)
\right]-R_w\omega_i
$$

$H\omega_K=b$へ整理し、

$$
\omega_{K,kin}=(H^TH+10^{-8}I)^{-1}H^Tb
$$

$$
\hat\omega_{K,k+1}=\hat\omega_{K,k}
+\frac{T_s}{\tau_K+T_s}
(\omega_{K,kin}-\hat\omega_{K,k}),\qquad\tau_K=0.030\ \mathrm{s}
$$

### 5.5 機体–ボール相対変位

$$
{}^W\dot p_{B/K,xy}={}^Wv_{B,xy}-{}^Wv_{K,xy}
$$

$$
{}^Bp_{B/K,xy,k+1}=R_z(\hat\psi)^T
\left[
0.9998R_z(\hat\psi){}^Bp_{B/K,xy,k}
+T_s({}^Wv_{B,xy}-{}^Wv_{K,xy})
\right]
$$

### 5.6 低運動認定とヨー軸ジャイロバイアス

エンコーダーは絶対ヨー角の観測ではなく、バイアスを学習してよい低運動区間の認定に使用する。走行中に車輪回転だけから機体ヨー角速度とボール角速度を常時分離できるとは仮定しない。

未認定時の候補条件は次の論理積とする。

$$
\max_i|\omega_{w,i}|\le\omega_{w,th}
$$

$$
|\|f^B\|-g|\le a_{th}
$$

$$
\sqrt{\bar p^2+\bar q^2}\le\omega_{rp,th}
$$

$$
|r_{IMU}-\hat b_{g,z}|\le\omega_{z,th}
$$

全条件の論理積を$Q_k$とし、継続時間を明示状態として更新する。

$$
t_{qual,k+1}=
\begin{cases}
\min(t_{qual,k}+T_s,t_{min}) & Q_k=1\\
0 & Q_k=0
\end{cases}
$$

$$
\gamma_k=Q_k\land(t_{qual,k}\ge t_{min})
$$

認定後は、角速度・比力閾値を1.25倍とする退出側ヒステリシスを適用する。これにより微小な量子化・振動で学習がチャタリングしにくくなる。退出条件を超えたサンプルでは$\gamma_k=0$としてバイアスを保持し、$t_{qual,k+1}=0$とする。

$$
\alpha_b=\frac{T_s}{\tau_b+T_s}
$$

$$
\Delta b_k=\operatorname{sat}\left(
\gamma_k\alpha_b(r_{IMU,k}-\hat b_{g,z,k}),
\pm\Delta b_{max}\right)
$$

$$
\hat b_{g,z,k+1}=\operatorname{sat}\left(
\hat b_{g,z,k}+\Delta b_k,\pm b_{max}\right)
$$

| パラメーター | 暫定値 | 根拠 |
|---|---:|---|
| $\omega_{w,th}$ | 0.10 rad/s | 輪周速度2.4 mm/s相当の低運動ゲート |
| $a_{th}$ | $0.03g$ | 静止時の小振動を許容する初期値 |
| $\omega_{rp,th}$ | 0.02 rad/s | 約1.15 deg/s以下を低運動とみなす初期値 |
| $\omega_{z,th}$ | 0.05 rad/s | 0.02 rad/s注入バイアスを学習範囲に含める初期値 |
| $t_{min}$ | 0.50 s | 一時停止・単発振動を除外する滞留時間 |
| $\tau_b$ | 1.0 s | 0.5 s認定後、モデルが低運動域を保つ間に0.02 rad/s注入を90%以上低減 |
| $b_{max}$ | 0.10 rad/s | 未確定IMUに対する保守的な異常上限 |
| $\Delta b_{max}$ | $1.0\times10^{-4}$ rad/s/sample | 単発外れ値による急変を制限 |

実機IMUのゼロレート出力、ノイズ密度、温度ドリフト、振動スペクトルは未確定であるため、全値を`ballbotParameters.m`の調整可能パラメーターとする。

### 5.7 サンプル内の計算順序

1. $\hat b_{g,z,k}$でIMU角速度を補正する。
2. 補正後角速度と加速度由来補正でクォータニオンを更新する。
3. 補正後角速度で車輪接触運動学を計算する。
4. 低運動候補、$t_{qual,k+1}$、$\gamma_k$を計算する。
5. $\hat b_{g,z,k+1}$を更新し、次サンプルから使用する。

この順序によりバイアス更新から当該サンプルの姿勢への直達を設けず、代数ループを作らない。

対応実装: `ballbotWheelRateFromDisplacement.m`、`ballbotEstimatorUpdate.m`→`ballbotEstimatorStep.m`。

## 6. 速度・姿勢・ヨー制御

正本モデル（rev2）の制御器は**トルク入力LQI**を中核とするマルチレート構成とする。外側 200 Hz タスク `ballbotLqiControllerUpdateRev2` が姿勢トルク・運動学的輪速目標・モード・コンテキストを生成し、内側 1 kHz タスク `ballbotWheelServoDutyStep` が推進輪速サーボとDCモータ式duty反転を実行する。

$$
\mathrm{output}_{outer}=
\begin{bmatrix}\tau_{att}(1{:}3)&I_v(4{:}5)&\mathrm{mode}(6)&s_{ready}(7)&\omega_{w,d}(8{:}10)&s_{settle}(11)&\mathrm{enable}(12)\end{bmatrix}^T
$$

### 6.1 モード管理

$$
\mathrm{mode}=
\begin{cases}
0 & \lnot\mathrm{enable}\ \lor\ \sqrt{\hat\phi^2+\hat\theta^2}\ge35\ \mathrm{deg}\ \lor\ \mathrm{latch}\\
2 & \sqrt{\hat\phi^2+\hat\theta^2}\ge5\ \mathrm{deg}\ \lor\ (\mathrm{prev}=2\ \land\ \mathrm{tilt}\ge2\ \mathrm{deg})\\
1 & \mathrm{otherwise}
\end{cases}
$$

mode 0 は**終端状態**（fallen→recovery 再突入は床バウンス反復フェースプラントを生じるため経路を持たない）であり、進入時にシミュレーションも打ち切る。mode 2 進入は `elevatedAuthority` をアームし、車輪がフィルタ済み輪速で 1.5 rad/s 未満へ静止するまで保持する。有効中はトルク上限を `recoveryWheelTorqueLimit`（公称 0.162 N·m）とし、mode 1 復帰後のセトル区間（`settleWitness`）では推定器のボール/機体速度チャネル（ドリフト中に ~7 倍/~3 倍のバイアスを実測）をエンコーダー輪速の運動学逆解（`pinv(wheelSpeedFromPlanarVelocity)`、0.25 s一次フィルタ）へ切替える。

### 6.2 指令成形

指令はワールド→機体変換の後、3段の制限を通す。

$$
\omega_{demand}=\sin\lambda\,\|{}^Bv_d\|+R_c|r_d|
\le R_w\,\omega_{cmd,max}
$$

結合リミッタ（`coupledCommandLimitEnabled`）が並進・ヨーの合成輪速要求を輪速予算内へ同率縮退し、`maxSpeed`（0.35 m/s）で速度を制限する。さらに**フォロースルーチェイス**を適用する: 指令速度がボール速度の `stopFollowGain`（0.75）倍未満のとき、輪速サーボへ与える指令を `followGain·v_{ball}` へ書換え、ボールとの共転を維持する（速度誤差フィードバックには書換え前のユーザー指令を残す）。settle 中は `settleFollowGain`=1.0 の純粋追従とする。

### 6.3 速度積分（休眠経路）

ボール速度誤差の条件付き積分状態 $I_v\in\mathbb{R}^2$（デッドゾーン・指令ゲート・リーク・非対称アンワインド）は簿記として保持するが、現行 `torqueLqi.integralGain` はゼロであり出力へ寄与しない。車輪トルク経路の積分は内力でボールを駆動できず、リーン需要経路の積分は蓄積バイアスで ~0.9 m/s 暴走を反復したため廃止し、定常抵抗は §6.5 の無状態ドラッグFFで賄う。

### 6.4 トルク入力LQI

フィードバック状態は10状態とする。

$$
x=\begin{bmatrix}
v_{ball,x}^B&v_{ball,y}^B&\theta&-\phi&q&-p&r&\omega_{w,1}&\omega_{w,2}&\omega_{w,3}
\end{bmatrix}^T
$$

$$
\tau_{att}=-K_{10}\,x
$$

| 状態要素 | 条件 |
|---|---|
| $v_{ball}^B$（1:2） | settle 中のみ $\hat v_{ball}^B-{}^Bv_d^{user}$、通常運転では0（通常時に有効化すると力チャネルが惰性走行ボールを牽引駆動する回帰のため） |
| $\theta,-\phi$（3:4） | リーン整定点・静的リーンバイアスを引いた傾斜誤差 |
| $q,-p,r$（5:7） | 20 ms一次遅れフィルタ済み機体角速度（gyroノイズ→トルクノイズ増幅の抑制）。$r$ はヨーゲート成立時のみ指令差を引く |
| $\omega_w$（8:10） | 30 ms一次遅れフィルタ済み輪速（量子化フリッカー ~3.7 rad/s対策。`wheelRateObserverEnabled=false` が既定 — オブザーバの driveGain=306 が荷重応答を ~100 倍過大予測し推進を抑制するため） |

ゲイン $K_{10}$ は `ballbotRobustTorqueGains` が設計するロバスト固定ゲインである。公称 0.5 kg ビルドで計測したトルクステップB行列（6×3、傾斜加速度・輪加速度行）をペイロード族 0–1 kg へスケーリング（傾斜行は $1/I_{pivot}$、ヨー行は $1/I_{yaw}$、輪行は不変、不安定傾斜係数は $mgh/I_{pivot}$）し、8状態内側プラント $[\theta,-\phi,q,-p,r,\omega_{w,1..3}]$ の離散LQR（$Q=\operatorname{diag}(200,200,20,20,5,0.2,0.2,0.2)$、$R=4000I_3$）を質量毎に解く。全候補ゲインを全プラントへ適用し、最悪閉ループ極半径が最小のゲイン（常に重端設計）を採用する。実装値は経験的列スケーリング（tilt×1.8、tilt-rate×1.0、yaw-rate×5）を適用済みで、速度状態2列はゼロである（`ballbotVelocityTorqueGains` が別設計）。輪速コラムへは -1.35/s のコンプライアンス減衰を加える。

素の状態フィードバックに加え、次の補助項をトルクへ重畳する。

| 項 | 仕様 |
|---|---|
| 傾き剛性スケジュール | `tiltStiffenGain`=2（`leanSetpointLimit`超過で`tiltStiffenTilt`=0.006 rad帯へ線形ブースト、settle中はエッジを`settleLeanCap`へ降下）。小信号は低ゲインでIMUノイズを抑え、コーン縁の大外乱は高剛性で捕捉する |
| 傾斜積分 | `tiltIntegralGain`=1（リーク0.5/s・上限0.03 rad、重端で比例ゲインが静的保持床の ~5% 上しかないため定常保持荷重を積分器が引き受ける） |
| 重力保持FF | `leanHoldTorque`×要求リーン角をトルクへFF（要求リーンの保持トルクを滑らかに供給し、フィードバックは偏差のみを扱う） |
| スリップ減衰 | `slipDampingGain`=0.02（フィルタ済み輪速−運動学転がりレートの滑り差分、デッドゾーン付き） |

RECOVERY（mode 2）ではリーン整定点を0へ強制して姿勢復帰のみを行い、mode 0 では全トルク0とする。

### 6.5 リーン要求外側ループ

車輪トルクは機体＋ボール系に対して内力でありアセンブリ重心を加速できないため、並進追従は重力を介したリーン駆動のみが外部推力として機能する。外側ループは速度誤差から傾斜整定点を生成し、状態の傾斜項へ注入する。

$$
a_{lean}=K_{v,lean}\,e_v^{user},\qquad
e_v^{user}={}^Bv_d^{user}-{}^B\hat v_{ball},\qquad
K_{v,lean}=0.30\ (settle中は1.0)
$$

| 項 | 仕様 |
|---|---|
| 加速度フィードフォワード | `accelFeedforwardGain`×指令加速度を片側加算（ボール先行時は遮断） |
| ドラッグフィードフォワード | `dragLeanFeedforward`×$r_f g$ を進行方向へ片側加算（無状態、積分器の暴走機構を持たない） |
| リーン上限 | `leanSetpointLimit`（0.022 rad、settle中は`settleLeanCap`=0.015） |
| スルー制限 | `leanRateLimit`×$g$ rad/s 相当（ステップ要求が支持を剥がす実績対策） |
| コーンフェード | 傾斜超過分に応じて需要を減衰（ブレーキ成分は殺さない片側適用） |
| 偏心ガード | `accelRelPosGuardGain`=12（ボール先行幾何への突入加速を絞る、フロア0.1） |
| コーストゲート | ボールが指令+`overspeedCoastMargin`（0.12 m/s、フロア0.5 m/s）を超えたとき運動方向成分を全剥がし（settle中は不発） |
| settle減衰 | `settleLeanDamping`=1 でフィルタ済みドリフト加速度を需要から減算 |

### 6.6 静的リーン適応（CGオフセット）

`staticLeanAdapt*`（既定ON、gain 0.5）が指令0時のドリフト速度シグネチャをローパス・デッドバンド(0.02)・積分し、リーン整定点バイアスとして `state(3:4)` へ注入する。真の平衡リーンで自己停止する負帰還とし、アーム条件・注入は指令0時のみ・成分クリップ±0.10 rad の安全性を備える。実測で 20 mm ロッドオフセット（CG横ずれ ~5.6 mm・平衡リーン ~2°）まで保持する。

### 6.7 ヨーチャネル

ヨーレート状態はコモンモードトルクがボールの自由zスピンをポンプするため**指令時のみ有効**とし、無指令時は状態を0へ強制する。無ヨー指令時は姿勢トルクのコモンモード残差（`rawWheelCommand-mean`）を除去し、受動ヨーダンパー（`yawDampingGain`=0.3、移動指令中のみ）とボールzスピンブレーキ（`ballYawGain`=0.12、トルク上限0.025 N·m）に任せる。起動時は `ballbotYawBiasStartupGuard` がバイアス収束までヨートルクだけを抑止し、明示指令は抑止をバイパスする。

### 6.8 内側 1 kHz 車輪サーボ

`ballbotWheelServoDutyStep`（`servoSampleTime`=1 ms）はZOHされた姿勢トルク・輪速目標・コンテキストを受け取る。

$$
\tau_{prop}=\operatorname{sat}\!\left(K_{prop}\,\operatorname{dz}(\omega_{w,d}-\hat\omega_w),\pm\tau_{prop,max}\right)
\ \ (\mathrm{mode}=1のみ、settle中0)
$$

$$
\tau=\operatorname{sat}(\tau_{att}+\tau_{prop},\pm\tau_{lim}),\qquad
\tau_{lim}=\min(\tau_{cont},\tau_{traction}/\tau_{recovery})
$$

$$
d=\operatorname{sat}\!\left(\frac{(R_a+R_{br})I+K_e\hat\omega_w}{V_s}+d_{comp},\ \pm d_{max}\right)
$$

| 項 | 値 |
|---|---:|
| 推進ゲイン $K_{prop}$ | 0.020 N·m/(rad/s) |
| 推進トルク上限 | 0.045 N·m（ヘッドルーム配分＋最小シェア0.25） |
| back-EMFフィルタ時定数 | 1 ms（5 ms計測窓の輪速を平滑） |
| トルク上限（通常） | $\min(0.919,0.162)=0.162$ N·m |
| トルク上限（リカバリ/セトル） | 0.162 N·m |
| 起動ランプ | 0.4 s（推進項のみ） |
| FR-16デッドゾーン補償 | `drive.deadzoneCompPos/Neg`（輪別・符号別同定値）、enable時かつmode≠0 |
| dutyクランプ | ±0.8333 |

### 6.9 起動時ヨー制御ガード

$$
s_{ready,k+1}=s_{ready,k}\lor
\left(\gamma_k=1\land|\hat r_k|\le0.002\ \mathrm{rad/s}\right)
$$

$$
r_{d,eff}=
\begin{cases}
\operatorname{sat}(r_d,\pm0.80) & s_{ready}\ \lor\ |r_d|>t_{bypass}\\
0 & \mathrm{otherwise}
\end{cases}
$$

明示的なヨー指令がない場合はヨー速度指令だけを0とし、未補正バイアスに制御器が反応して車輪を回し、低運動認定を自ら解除する競合を防ぐ。傾斜安定化と速度追従は常時有効である。一度成立した$s_{ready}$は推定器・制御器リセットまで保持する。明示指令では指令を優先するが、運動中のバイアス更新は低運動ゲートにより停止する。

### 6.10 代替・廃止経路

| 経路 | 状態 |
|---|---|
| `ballbotFullPlantHierarchyUpdate`（旧階層制御） | `p.controller.fullplant.enabled=true` で選択可能な実験経路。既定はトルクLQI |
| 輪速指令10状態LQI＋輪速PI（rev1経路） | `torqueDrive.enabled=false` のフォールバック。`ballbotDesignLqi` が縮約プラント線形化から設計 |
| 速度積分出力 | §6.3 の休眠経路（簿記のみ） |
| 輪速オブザーバ | `wheelRateObserverEnabled=false` が既定（荷重応答 ~100倍過大予測バグ） |
| デッドゾーンオンライン適応 | `ballbotDeadzoneAdaptStep` PoC 保存のみ・未接続 |
| cascade/ballforce 経路 | 全指標でトルクLQIに劣後したため削除済み |

対応実装: `ballbotLqiControllerUpdateRev2.m`、`ballbotWheelServoDutyStep.m`、`ballbotLqiControllerUpdate.m`、`ballbotLqiFeedbackState.m`、`ballbotRobustTorqueGains.m`、`ballbotDesignTorqueLqi.m`、`ballbotTorquePlantDynamics.m`、`ballbotVelocityTorqueGains.m`、`ballbotYawBiasStartupGuard.m`。

## 7. 可観測性・推定誤差

| モード | 観測される量 | 非可観測/弱可観測量 |
|---|---|---|
| 認定済み静止 | ロール、ピッチ、角速度、ヨー軸ジャイロバイアス | 絶対ヨー、絶対XY位置 |
| 理想転動・3輪接触 | 上記+球角速度、相対速度 | 一様な位置オフセット |
| すべり | IMU姿勢・角速度 | 球角速度と接触状態が弱可観測 |
| 分離 | IMU自由運動 | 球状態・相対接触位置 |

バイアス補正はヨードリフトを低減するが、絶対ヨーの観測を追加しない。実機化時に絶対ヨーを長期保持するための最小追加観測は絶対方位1量であり、絶対平面運動には速度または位置2量も必要である。

## 8. 実装対応表

| 理論節 | MATLAB関数 | Simulinkサブシステム |
|---|---|---|
| §2 | `ballbotWheelGeometry.m` | パラメーター初期化（`ballbotFullPlantParameters`から呼出し） |
| §2 | $W_\omega$生成 | `ballbotFullPlantParameters.m` |
| §3 | `ballbotCustomFriction.m`、`ballbotRollingResistance.m` | MultibodyPlant/WheelNCustomFriction→WheelAssemblyN/WheelNBallContact、Ball/BallRollResFcn |
| §4 | `ballbotImuFromJoint.m`→`ballbotIdealImu.m` | MultibodyPlant/IdealIMU |
| §5.4 | `ballbotWheelRateFromDisplacement.m` | Controller/WheelSpeedSensing、ServoWheelSpeedSensing/WheelRateCalculation |
| §5 | `ballbotEstimatorUpdate.m`→`ballbotEstimatorStep.m` | Controller/StateEstimator/EstimatorUpdate |
| §6.1–6.7, 6.9 | `ballbotLqiControllerUpdateRev2.m` | Controller/BalanceController/LqiController（5 ms） |
| §6.8 | `ballbotWheelServoDutyStep.m` | Controller/WheelDrive/SpeedPiDuty（1 ms） |
| §6.10 | `ballbotLqiControllerUpdate.m`→`ballbotFullPlantHierarchyUpdate.m`、`ballbotDesignLqi.m` | 代替経路（実験用） |
| §6.4 ゲイン設計 | `ballbotRobustTorqueGains.m`、`ballbotDesignTorqueLqi.m`、`ballbotTorquePlantDynamics.m` | パラメーター初期化 |
| 駆動・補償 | `ballbotMdd3aGateDrive.m`、`ballbotShaftFrictionTorque.m` | MultibodyPlant/WheelAssemblyN |
| 全パラメーター | `ballbotRev1Parameters.m`（`ballbotFullPlantParameters.m`→`ballbotParameters.m`、実行時付加は`ballbotRev1PlantAugment.m`） | Model workspace `ballbotParams` |

## 9. 参考資料

| 資料 | 採用内容 |
|---|---|
| [MathWorks: Spatial Contact Force](https://www.mathworks.com/help/sm/ref/spatialcontactforce.html) | 法線ペナルティ、摩擦入力、分離、接触量 |
| [Lalほか, 2019](https://busoniu.net/files/papers/ddecs19.pdf) | 3オムニホイール球駆動、LQR設計の基礎構成 |
| [Lalほか, 2020](https://busoniu.net/files/papers/ifac20-ioana.pdf) | $D(q)\ddot q+C(q,\dot q)\dot q+G(q)=B\tau$、3輪トルク変換 |
| [Mucchiani, 2018](https://escholarship.org/uc/item/2cd979dq) | 高ヨー速度時の非線形球乗りロボット、EKF・モデルベース制御 |

## 10. 関連文書

- [システム仕様](ball_balancing_omnirover3wd-system.md)
- [アーキテクチャ](ball_balancing_omnirover3wd-architecture.md)
- [状態方程式](ball_balancing_omnirover3wd-state-equations.md)
- [検証計画・ロバスト性受け入れ試験](ball_balancing_omnirover3wd-test-plan.md)
- [仕様変更記録](ball_balancing_omnirover3wd-change-history.md)
