# 玉乗り3WDオムニローバー制御・状態推定理論

## 状態

| 項目 | 値 |
|---|---|
| ステータス | 実装一致版 |
| 最終更新日 | 2026-09-01 |
| 実装 | `matlab_ws/ball_balancing_omni3/*.m` |

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
\beta_i\in\{0,2\pi/3,4\pi/3\},\qquad\lambda=\pi/4
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

上式は3輪トルクがボールへ及ぼす一般化トルクの物理関係である。現行の制御器はトルク配分ではなく輪速空間で指令を生成するため、速度指令への対応として同じ幾何から

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
| ホイール–ボール | $2.0\times10^5$ | 250 | $5.0\times10^{-4}$ |
| ボール–床 | $3.0\times10^5$ | 180 | $5.0\times10^{-4}$ |

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
| $\mu_d$ | 0.75 |
| $\mu_r$ | 0.02 |
| $v_c$ | 0.005 m/s |

対応実装: `ballbotCustomFriction.m`。

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
({}^B\hat p_{B/K,xy})^T&c_{contact}
\end{bmatrix}^T
$$

外部推定出力$\hat z\in\mathbb{R}^{14}$の幅と順序は維持する。角速度$[\hat p,\hat q,\hat r]^T$は生ジャイロ値ではなく、前回サンプルまでの$\hat b_{g,z}$を差し引いた値とする。接触信頼度は引き続き14番目とする。

診断出力は次の3要素を別信号で公開する。

$$
d_b=\begin{bmatrix}\hat b_{g,z,k+1}&\gamma_k&t_{qual,k+1}\end{bmatrix}^T
$$

### 5.2 姿勢更新

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

### 5.6 接触信頼度

$$
\epsilon_{RMS}=\sqrt{\frac{1}{3}\sum_{i=1}^3\epsilon_i^2},\qquad
c_{contact}=\exp\left[-(\epsilon_{RMS}/0.25)^2\right]
$$

### 5.7 低運動認定とヨー軸ジャイロバイアス

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

$$
c_{contact}\ge c_{th}
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

認定後は、角速度・比力閾値を1.25倍、接触信頼度閾値を0.70とする退出側ヒステリシスを適用する。これにより微小な量子化・振動で学習がチャタリングしにくくなる。退出条件を超えたサンプルでは$\gamma_k=0$としてバイアスを保持し、$t_{qual,k+1}=0$とする。

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
| $c_{th}$ | 0.80 | 公称接触に限定する初期値 |
| $t_{min}$ | 0.50 s | 一時停止・単発振動を除外する滞留時間 |
| $\tau_b$ | 1.0 s | 0.5 s認定後、モデルが低運動域を保つ間に0.02 rad/s注入を90%以上低減 |
| $b_{max}$ | 0.10 rad/s | 未確定IMUに対する保守的な異常上限 |
| $\Delta b_{max}$ | $1.0\times10^{-4}$ rad/s/sample | 単発外れ値による急変を制限 |

実機IMUのゼロレート出力、ノイズ密度、温度ドリフト、振動スペクトルは未確定であるため、全値を`ballbotParameters.m`の調整可能パラメーターとする。

### 5.8 サンプル内の計算順序

1. $\hat b_{g,z,k}$でIMU角速度を補正する。
2. 補正後角速度と加速度由来補正でクォータニオンを更新する。
3. 補正後角速度で車輪接触運動学と接触信頼度を計算する。
4. 低運動候補、$t_{qual,k+1}$、$\gamma_k$を計算する。
5. $\hat b_{g,z,k+1}$を更新し、次サンプルから使用する。

この順序によりバイアス更新から当該サンプルの姿勢・接触信頼度への直達を設けず、代数ループを作らない。

対応実装: `ballbotWheelRateFromDisplacement.m`、`ballbotEstimatorUpdate.m`→`ballbotEstimatorStep.m`。

## 6. 速度・姿勢・ヨー制御

正本モデルの制御器は階層構造とし、`ballbotLqiControllerUpdate` が `p.controller.fullplant.enabled` を検査して `ballbotFullPlantHierarchyUpdate` へ委譲する。外側層は輪速指令$\omega_{w,d}$を生成し、内側層 `ballbotSpeedPiMotorStep` が輪速PIとDCモータモデルで輪トルクへ変換する。

### 6.1 モード管理と指令成形

$$
\mathrm{mode}=
\begin{cases}
0 & \lnot\mathrm{enable}\ \lor\ \sqrt{\hat\phi^2+\hat\theta^2}\ge35\ \mathrm{deg}\\
2 & \sqrt{\hat\phi^2+\hat\theta^2}\ge17\ \mathrm{deg}\ \lor\ c_{contact}<c_{min}\\
1 & \mathrm{otherwise}
\end{cases}
$$

正本構成では`p.controller.minimumContactConfidence=0`として接触信頼度ゲートを無効化し、傾斜だけでRECOVERYへ進入する。

$$
{}^Bv_d=c_{scale}R_z(\hat\psi)^T{}^Wv_d,\qquad
\|{}^Bv_d\|\le p.\mathrm{controller.maxSpeed}
$$

指令スケール$c_{scale}=0.60$は、輪速変換を介してモータ速度指令が物理上限へ達するまでの余裕を確保する。

### 6.2 機体平面速度の再構成

階層制御ではIMU積分速度ではなく、車輪角速度から機体平面速度を再構成する。

$$
{}^B\hat v=\left(W_{\omega}(:,1{:}2)\right)^\dagger\omega_w
$$

エンコーダー由来の輪速を直接使うため、比力積分のドリフトが速度外側ループへ入らない。

### 6.3 速度外側ループと傾斜指令

$$
e_v={}^Bv_d-{}^B\hat v
$$

$$
I_{v,k+1}=\operatorname{sat}(I_{v,k}+T_se_v,\pm0.20)
$$

$$
a_d=K_{p,v}e_v+K_{i,v}I_v,\qquad\|a_d\|\le0.60\ \mathrm{m/s^2}
$$

$$
\alpha_d=
\begin{bmatrix}\operatorname{atan2}(a_{d,x},g)\\ \operatorname{atan2}(a_{d,y},g)\end{bmatrix},
\qquad\|\alpha_d\|_\infty\le4\ \mathrm{deg}
$$

| ゲイン | 値 |
|---|---:|
| $K_{p,v}$ | $\operatorname{diag}(2.3,2.3)$ s$^{-1}$ |
| $K_{i,v}$ | $\operatorname{diag}(0,0)$ s$^{-2}$ |

### 6.4 傾斜安定化と輪速指令

傾斜状態は$[\theta;-\phi]$、傾斜角速度は$[q;-p]$で統一する。

$$
u_{planar}=s_{stab}\left\{
K_{p,\alpha}(\alpha-\alpha_d)+K_{d,\alpha}\dot\alpha
\right\}-K_{ff}\,{}^Bv_d
$$

| パラメーター | 値 |
|---|---:|
| $K_{p,\alpha}$ | $\operatorname{diag}(16,16)$ |
| $K_{d,\alpha}$ | $\operatorname{diag}(1.6,1.6)$ |
| $s_{stab}$ | $+1$ |
| $K_{ff}$ | $1.0$ |

RECOVERYでは$\alpha_d=0$、FALLEN/DISABLEDでは$u_{planar}=0$とする。

### 6.5 起動時ヨー制御ガード

$$
s_{ready,k+1}=s_{ready,k}\lor
\left(\gamma_k=1\land|\hat r_k|\le0.002\ \mathrm{rad/s}\right)
$$

$$
r_{d,eff}=
\begin{cases}
\operatorname{sat}(r_d,\pm0.80) & s_{ready}\ \lor\ |r_d|>1.0\times10^{-6}\\
0 & \mathrm{otherwise}
\end{cases}
$$

明示的なヨー指令がない場合はヨー速度指令だけを0とし、未補正バイアスに制御器が反応して車輪を回し、低運動認定を自ら解除する競合を防ぐ。傾斜安定化と速度追従は常時有効である。一度成立した$s_{ready}$は推定器・制御器リセットまで保持する。明示指令では指令を優先するが、運動中のバイアス更新は低運動ゲートにより停止する。

### 6.6 輪速変換と飽和

$$
\omega_{w,d}=\operatorname{sat}\!\left(
W_\omega\begin{bmatrix}u_{planar}\\ r_{d,eff}\end{bmatrix},
\ \pm\,0.70\,\omega_{max,motor}\right)
$$

$\omega_{max,motor}=$ `p.motor.speedCommandLimit` $=d_{max}\omega_{nl}$（$d_{max}$はMDD3A最大デューティ、$\omega_{nl}=210$ rpmの無負荷速度）。

| 条件 | $I_v$更新 |
|---|---|
| $\omega_{w,d}$非飽和かつBALANCE | 候補値を採用 |
| いずれかの輪速が飽和 | 前回値を保持 |
| mode≠BALANCE | 0 |

### 6.7 輪速内側ループとモータ

`ballbotSpeedPiMotorStep` は輪速PI、MDD3A電圧制限、DCモータ電気式、トルク制限を順に適用する。

$$
e_\omega=\omega_{w,d}-\omega_w
$$

$$
V_{raw}=K_{p,\omega}e_\omega+K_{i,\omega}I_{\omega,k},\qquad
V=\operatorname{sat}(V_{raw},\pm d_{max}V_{supply})
$$

$$
I_{\omega,k+1}=\operatorname{sat}\!\left(
I_{\omega,k}+T_s\big(e_\omega+K_{aw,\omega}(V-V_{raw})\big),
\pm\frac{V_{supply}}{K_{i,\omega}}\right)
$$

電圧飽和はバックカリキュレーション（$K_{aw,\omega}=1/K_{p,\omega}$）で積分器へ戻す。$V_{supply}=6$ V、$d_{max}$はMDD3A最大デューティ。mode=0では電圧と積分器を0へリセットする。

$$
I=\operatorname{sat}\!\left(\frac{V-K_e\omega_w}{R_a},\pm I_{cont}\right),\qquad
I_{cont}=3.0\ \mathrm{A}
$$

$$
\tau_w=\operatorname{sat}\!\left(K_tI-B\omega_w,\pm\tau_{lim}\right),\qquad
\tau_{lim}=\min(\tau_{traction},\tau_{motor})
$$

正本構成では$\tau_{motor}=$ `p.controller.fullplant.motorTorqueLimit` $=0.010$ N·mを輪トルク上限とし、接触摩擦限界$\tau_{traction}=\mu_d N_{nom}R_w$とMDD3A連続定格トルクの小さい方と併せて制限する。輪速PIゲインは正本構成で$\times0.25$に再調整する（`p.fullplant.speedPiGainScale`）。

### 6.8 代替パス：10状態MIMO LQI

`p.controller.fullplant.enabled=false` の構成では、`ballbotLqiControllerUpdate` 内部の10状態LQIを直接適用する。

$$
x=\begin{bmatrix}{}^Bv^T&\theta&-\phi&q&-p&r&\omega_w^T\end{bmatrix}^T\in\mathbb{R}^{10}
$$

$$
\omega_{w,d}=\operatorname{sat}\!\left(
W_\omega\begin{bmatrix}0\\0\\r_{d,eff}\end{bmatrix}
-Kx-\lambda_K K_i I_v,\ \pm\omega_{max,motor}\right)
$$

$$
I_v^+=I_v+T_se_v+T_sK_{aw}\,
\left[W_\omega^\dagger(\omega_{w,d}-\omega_{w,raw})\right]_{1:2}
$$

ヨーは運動学的フィードフォワードのみとし、平面追従は積分状態経由で入れるため、速度ステップが加速度・傾斜制限を迂回しない。飽和差分は$W_\omega$の疑似逆行列で平面速度空間へ戻し、$K_{aw}=8$で積分器を補正する。RECOVERYでは$\omega_{w,d}=-1.35Kx$、FALLEN/DISABLEDでは0とする。ゲイン$K,K_i$は`ballbotDesignLqi.m`が縮約プラントの線形化モデルから積分拡張LQRで設計する。

対応実装: `ballbotLqiControllerUpdate.m`、`ballbotFullPlantHierarchyUpdate.m`、`ballbotLqiFeedbackState.m`、`ballbotSpeedPiMotorStep.m`、`ballbotMdd3aVoltageController.m`、`ballbotYawBiasStartupGuard.m`、`ballbotDesignLqi.m`。

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
| §3 | `ballbotCustomFriction.m` | MultibodyPlant/WheelNCustomFriction→WheelAssemblyN/WheelNBallContact |
| §4 | `ballbotImuFromJoint.m`→`ballbotIdealImu.m` | MultibodyPlant/IdealIMU |
| §5.4 | `ballbotWheelRateFromDisplacement.m` | Controller/WheelSpeedSensing/WheelRateCalculation |
| §5 | `ballbotEstimatorUpdate.m`→`ballbotEstimatorStep.m` | Controller/StateEstimator/EstimatorUpdate |
| §6.1–6.6 | `ballbotLqiControllerUpdate.m`→`ballbotFullPlantHierarchyUpdate.m` | Controller/BalanceController/LqiController |
| §6.7 | `ballbotSpeedPiMotorStep.m`→`ballbotMdd3aVoltageController.m` | Controller/WheelDrive/SpeedPiAndDcMotor |
| 全パラメーター | `ballbotFullPlantParameters.m`（基礎値は`ballbotParameters.m`） | Model workspace `ballbotParams` |

## 9. 参考資料

| 資料 | 採用内容 |
|---|---|
| [MathWorks: Spatial Contact Force](https://www.mathworks.com/help/sm/ref/spatialcontactforce.html) | 法線ペナルティ、摩擦入力、分離、接触量 |
| [Lalほか, 2019](https://busoniu.net/files/papers/ddecs19.pdf) | 3オムニホイール球駆動、LQR設計の基礎構成 |
| [Lalほか, 2020](https://busoniu.net/files/papers/ifac20-ioana.pdf) | $D(q)\ddot q+C(q,\dot q)\dot q+G(q)=B\tau$、3輪トルク変換 |
| [Mucchiani, 2018](https://escholarship.org/uc/item/2cd979dq) | 高ヨー速度時の非線形球乗りロボット、EKF・モデルベース制御 |
