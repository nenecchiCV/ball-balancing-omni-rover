# 玉乗り3WDオムニローバー制御・状態推定理論

## 状態

| 項目 | 値 |
|---|---|
| ステータス | 実装一致版 |
| 最終更新日 | 2026-08-31 |
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
\tau_b^B=A_\tau\tau_w,qquad
A_\tau=\frac{R_b}{R_w}
\begin{bmatrix}a_1^B&a_2^B&a_3^B\end{bmatrix}
$$

実装は正則化疑似逆行列を用いる。

$$
\tau_{w,raw}=A_\tau^T(A_\tau A_\tau^T+10^{-10}I)^{-1}\tau_{b,d}
$$

$$
\tau_w=\operatorname{sat}(\tau_{w,raw},\pm0.0384\ \mathrm{N\,m})
$$

対応実装: `ballbotWheelGeometry.m`、`ballbotTorqueAllocator.m`。

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

対応実装: `ballbotIdealImu.m`。

## 5. 推定器

### 5.1 状態・出力

$$
x_e=
\begin{bmatrix}
q_{WB}^T&({}^Wv_B)^T&({}^W\omega_K)^T&({}^Bp_{B/K,xy})^T
\end{bmatrix}^T\in\mathbb{R}^{12}
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

### 5.2 姿勢更新

$$
u_g^B=R_{WB}^Te_3,\qquad
u_a^B=\frac{f^B}{\|f^B\|}
$$

$$
\omega_c^B=\omega_{IMU}^B+K_a(u_a^B\times u_g^B)
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
{}^Wv_{B,k+1}=\alpha_v{}^Wv_{B,k}+T_s{}^Wa_B,qquad
\alpha_v=0.9995
$$

平面モデルでは$v_{B,z}=0$を課す。

### 5.4 ボール角速度

エンコーダーの直接観測量は車輪回転変位$\theta_{w,i}$とし、推定器とサーボ速度包絡線で用いる車輪角速度は5 ms周期の後退差分で算出する。初回サンプルでは前回変位を現在変位で初期化し、$\omega_{w,i}[0]=0$とする。

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
c_{contact}=\exp\left[-(\epsilon_{RMS}/0.025)^2\right]
$$

対応実装: `ballbotWheelRateFromDisplacement.m`、`ballbotEstimatorStep.m`。

## 6. 速度・姿勢・ヨー制御

### 6.1 指令制限

$$
\|v_d^W\|\le0.12\ \mathrm{m/s},\qquad
|r_d|\le0.80\ \mathrm{rad/s}
$$

### 6.2 速度外側ループ

$$
e_v^B=R_z(\hat\psi)^T(v_d^W-\hat v_B^W)
$$

$$
I_{v,k+1}=\operatorname{sat}(I_{v,k}+T_se_v^B,\pm0.50)
$$

$$
a_d^B=K_{pv}e_v^B+K_{iv}I_v,\qquad\|a_d^B\|\le2.0\ \mathrm{m/s^2}
$$

$$
\phi_d=-\operatorname{atan2}(a_{d,y}^B,g),\qquad
\theta_d=\operatorname{atan2}(a_{d,x}^B,g)
$$

$$
|\phi_d|,|\theta_d|\le10\ \mathrm{deg}
$$

| ゲイン | 値 |
|---|---:|
| $K_{pv}$ | $\operatorname{diag}(2.0,2.0)$ s$^{-1}$ |
| $K_{iv}$ | $\operatorname{diag}(0.35,0.35)$ s$^{-2}$ |

### 6.3 姿勢内側ループ

$$
\tau_{b,x}=-s_m\{0.95(\hat\phi-\phi_d)+0.12\hat p\}
$$

$$
\tau_{b,y}=-s_m\{0.95(\hat\theta-\theta_d)+0.12\hat q\}
$$

$$
\tau_{b,z}=0.08(r_d-\hat r)
$$

| モード | $s_m$ | $(\phi_d,\theta_d,r_d)$ |
|---|---:|---|
| BALANCE | 1.00 | 速度外側ループの値 |
| RECOVERY | 1.35 | $(0,0,0)$ |
| FALLEN/DISABLED | - | $\tau_b=0$ |

### 6.4 飽和・アンチワインドアップ

$$
\tau_w=\operatorname{sat}(A_\tau^\dagger\tau_b,\pm0.0384\ \mathrm{N\,m})
$$

| 条件 | $I_v$更新 |
|---|---|
| $\tau_{w,raw}=\tau_w$ | 候補値を採用 |
| $\tau_{w,raw}\ne\tau_w$ | 前回値を保持 |
| mode≠BALANCE | 0 |

16007サーボの加速方向トルクには、最高回転速度
$\omega_{max}=62\times2\pi/60$ rad/sで0となる線形包絡線を適用する。減速方向トルクは接触トルク上限まで許容する。

対応実装: `ballbotControlStep.m`、`ballbotTorqueAllocator.m`、`ballbotServoTorqueEnvelope.m`。

## 7. 可観測性・推定誤差

| モード | 観測される量 | 非可観測/弱可観測量 |
|---|---|---|
| 理想静止 | ロール、ピッチ、角速度 | 絶対ヨー、絶対XY位置 |
| 理想転動・3輪接触 | 上記+球角速度、相対速度 | 一様な位置オフセット |
| すべり | IMU姿勢・角速度 | 球角速度と接触状態が弱可観測 |
| 分離 | IMU自由運動 | 球状態・相対接触位置 |

実機化時の最小追加観測は、絶対方位1量と平面速度または位置2量である。

## 8. 実装対応表

| 理論節 | MATLAB関数 | Simulinkサブシステム |
|---|---|---|
| §2 | `ballbotWheelGeometry.m` | ParameterInitialization |
| §2 | `ballbotTorqueAllocator.m` | Controller/TorqueAllocator |
| §3 | `ballbotCustomFriction.m` | MultibodyPlant/WheelBallContact_1..3 |
| §4 | `ballbotIdealImu.m` | MultibodyPlant/Sensors/Ideal6AxisIMU |
| §5.4 | `ballbotWheelRateFromDisplacement.m` | Controller/WheelRateDerivative |
| §5 | `ballbotEstimatorStep.m` | Controller/StateEstimator |
| §6 | `ballbotControlStep.m` | Controller/ModeAndControl |
| 全パラメーター | `ballbotParameters.m` | Model workspace `ballbotParams` |

## 9. 参考資料

| 資料 | 採用内容 |
|---|---|
| [MathWorks: Spatial Contact Force](https://www.mathworks.com/help/sm/ref/spatialcontactforce.html) | 法線ペナルティ、摩擦入力、分離、接触量 |
| [Lalほか, 2019](https://busoniu.net/files/papers/ddecs19.pdf) | 3オムニホイール球駆動、LQR設計の基礎構成 |
| [Lalほか, 2020](https://busoniu.net/files/papers/ifac20-ioana.pdf) | $D(q)\ddot q+C(q,\dot q)\dot q+G(q)=B\tau$、3輪トルク変換 |
| [Mucchiani, 2018](https://escholarship.org/uc/item/2cd979dq) | 高ヨー速度時の非線形球乗りロボット、EKF・モデルベース制御 |
