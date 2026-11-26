# 玉乗り3WDオムニローバー状態方程式

## 1. 対象と結論

車輪軸トルクを入力とし、ボールとローバーの位置・姿勢および車輪回転変位を出力とするプラントを、非線形記述系（DAE）として定義する。正本モデル rev2 は車輪トルクをFIT0521 DCモータ＋MDD3AドライバのSimscape Electrical回路（PWM duty入力）から生成するが、本書では電気系を含まない力学系の記述に集中し、duty→トルク変換は [アーキテクチャ §4.2](ball_balancing_omnirover3wd-architecture.md) に委ねる。

車輪入力3量だけでは、ボール角速度3量とローバー角速度3量を運動学だけから一意に分離できない。このため、絶対運動を表すプラントには質量・慣性・重力・接触反力を含む運動方程式が必要である。車輪速度やトルクから姿勢までを直接写像する陽な運動学モデルは、この機構の自由度を失うため採用しない。

## 2. 座標系と記号

| 記号 | 定義 | 単位 |
|---|---|---|
| $W$ | 慣性ワールド座標系、$+Z_W$上向き | - |
| $K$ | ボール固定座標系 | - |
| $B$ | ローバー機体座標系 | - |
| $p_K^W,p_B^W$ | ボール中心、ローバー基準点のワールド位置 | m |
| $q_{WK},q_{WB}$ | $K,B$から$W$への単位クォータニオン | - |
| $R_{WK},R_{WB}$ | 各クォータニオンに対応する回転行列 | - |
| $v_K^W,v_B^W$ | ボール中心、ローバー基準点の並進速度 | m/s |
| $\omega_K^K,\omega_B^B$ | ボール、ローバーの物体固定角速度 | rad/s |
| $\theta_w$ | 3輪の回転変位 | rad |
| $u=\tau_w$ | 入力となる3輪の車軸トルク | N·m |

クォータニオンはスカラー先頭の$q=[q_0,q_1,q_2,q_3]^T$とする。車輪番号は$eta_i=[0,120,240]$ degの順とし、正回転方向は既存実装の$+a_i^B$に合わせる。

## 3. 状態、入力、出力

最小座標への解析的消去を行わず、接触拘束を明示するため、状態を次のように置く。

$$
x=
\begin{bmatrix}
(p_K^W)^T&q_{WK}^T&(p_B^W)^T&q_{WB}^T&\theta_w^T&
(v_K^W)^T&(\omega_K^K)^T&(v_B^W)^T&(\omega_B^B)^T
\end{bmatrix}^T\in\mathbb{R}^{29}
$$

$$
u=\tau_w=
\begin{bmatrix}\tau_{w,1}&\tau_{w,2}&\tau_{w,3}\end{bmatrix}^T
\in\mathbb{R}^{3}
$$

トルク入力の場合、$\theta_w$ は拘束を受けない通常の自由度として運動方程式で時間発展する。理想速度源を用いる設計モデルでは $u=\dot\theta_w=\omega_w$ とし、§6 の $S_w\nu-u=0$ を車輪軸速度拘束として課す。

指定された出力は、姿勢をZYXオイラー角で表すと次の15要素である。

$$
y=
\begin{bmatrix}
(p_K^W)^T&\eta_K^T&(p_B^W)^T&\eta_B^T&\theta_w^T
\end{bmatrix}^T,qquad
\eta_j=\operatorname{rpy}_{ZYX}(q_{Wj})
$$

内部状態には特異点のないクォータニオンを使い、表示・ログ時だけロール・ピッチ・ヨーへ変換する。姿勢出力にも特異点回避が必要な用途では、$\eta_K,\eta_B$をクォータニオンへ置き換え、出力を17要素とする。

## 4. 運動学状態方程式

並進および車輪変位は、

$$
\dot p_K^W=v_K^W,qquad
\dot p_B^W=v_B^W,qquad
\dot\theta_w=\omega_w
$$

とする。姿勢は、

$$
\dot q_{Wj}=\frac{1}{2}\Omega(\omega_j^j)q_{Wj},qquad j\in\{K,B\}
$$

$$
\Omega(\omega)=
\begin{bmatrix}
0&-\omega_x&-\omega_y&-\omega_z\\
\omega_x&0&\omega_z&-\omega_y\\
\omega_y&-\omega_z&0&\omega_x\\
\omega_z&\omega_y&-\omega_x&0
\end{bmatrix}
$$

で更新し、$q_{Wj}^Tq_{Wj}=1$を保つ。

一般化座標と一般化速度を、

$$
q=\begin{bmatrix}p_K^W&q_{WK}&p_B^W&q_{WB}&\theta_w\end{bmatrix},qquad
\nu=\begin{bmatrix}v_K^W&\omega_K^K&v_B^W&\omega_B^B&\dot\theta_w\end{bmatrix}^T
$$

とまとめると、上式は$\dot q=T(q)\nu$と記述できる。

## 5. 接触拘束

### 5.1 ボールと床

床との接触維持は、

$$
\phi_g(q)=e_3^Tp_K^W-R_b=0
$$

である。床上で無すべりと仮定する場合、水平接線速度は、

$$
P_{xy}\left[v_K^W+\left(R_{WK}\omega_K^K\right)\times(-R_be_3)\right]=0,qquad
P_{xy}=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}
$$

を満たす。

### 5.2 車輪とボール

接触法線と駆動転動方向を機体座標で、

$$
n_i^B=
\begin{bmatrix}
\cos\lambda\cos\beta_i\\
\cos\lambda\sin\beta_i\\
\sin\lambda
\end{bmatrix},qquad
t_i^B=
\begin{bmatrix}-\sin\beta_i&\cos\beta_i&0\end{bmatrix}^T
$$

とする。$r_{F_i/B}^B$を機体基準点から車輪接触点への位置、$n_i^W=R_{WB}n_i^B$、$t_i^W=R_{WB}t_i^B$とすると、駆動方向の無すべり条件は、

$$
(t_i^W)^T\left[
v_B^W+(R_{WB}\omega_B^B)\times(R_{WB}r_{F_i/B}^B)
-v_K^W-(R_{WK}\omega_K^K)\times(R_bn_i^W)
\right]-R_wu_i=0
$$

である。オムニホイールのローラー方向には拘束を置かない。3個の法線接触は、車輪中心位置$p_{C_i}^W(q)$を用いて、

$$
\phi_i(q)=\|p_{C_i}^W(q)-p_K^W\|-(R_b+R_w-\delta_p)=0
$$

とする。$\delta_p$は数値的な接触維持に用いる微小プリロードである。

これらをまとめて、

$$
\Phi_n(q)=0,qquad J_t(q)\nu-D_wu=0
$$

と表す。$\Phi_n=[\phi_g,\phi_1,\phi_2,\phi_3]^T$、$J_t$は床2方向と車輪3方向の計5本の接線拘束、$D_w$は各車輪速度を$R_wu_i$へ写像する行列である。

## 6. 非線形記述状態方程式

質量・慣性と接触反力を含むプラントは、次のDAEで表す（トルク入力版）。

$$
\boxed{
\begin{aligned}
\dot q &= T(q)\nu\\
M(q)\dot\nu+h(q,\nu) &= S_w^T\tau_w+J_n(q)^T\lambda_n+J_t(q)^T\lambda_t\\
0&=\Phi_n(q)\\
0&=J_t(q)\nu-D_w\omega_w
\end{aligned}}
$$

| 項 | 内容 |
|---|---|
| $M(q)$ | ボール、機体、モータポッド、車輪を含む一般化質量行列 |
| $h(q,\nu)$ | コリオリ・遠心項、重力、受動摩擦（輪軸Coulomb/静摩擦・球転がり抵抗） |
| $S_w$ | 一般化速度から車輪軸速度を選ぶ行列 |
| $\tau_w$ | 駆動系が印加する車輪軸トルク（rev2ではduty→DCモータ式で生成） |
| $J_n^T\lambda_n$ | 床1点と車輪3点の法線反力 |
| $J_t^T\lambda_t$ | 無すべり時の接線反力 |

理想速度源による設計モデルでは入力を $u=\dot\theta_w$ として $0=S_w\nu-u$（車輪軸速度拘束）を加え、$\tau_a$ は拘束維持のため発生する代数トルクとなる。$J_t\nu-D_w\omega_w=0$の車輪3行は接触点速度拘束であり、速度拘束モデルでは $S_w\nu-u=0$ とともに用いる（同じ速度条件を二重に課さない）。

初期値は$\Phi_n(q_0)=0$と速度拘束を満たす整合初期値でなければならない。速度拘束モデルで速度指令が不連続な場合、理想速度源はインパルス状トルクを要求するため、実装では車輪速度指令に一次遅れまたは加速度制限を加える（正本のトルク入力経路ではこの問題は生じない）。

### 6.1 ペナルティ接触を使う正本Simscapeモデル

既存モデルと同じ有限剛性接触を用いる場合は、$\Phi_n=0$と接触反力$\lambda$を代数拘束から外し、貫入量$d$と相対速度から計算する接触力$F_c$に置き換える。

$$
M(q)\dot\nu+h(q,\nu)=S_w^T\tau_a+J_c(q)^TF_c(q,\nu,u)
$$

$$
F_n=s(d,w)\max(k_nd+c_n\dot d,0)
$$

$$
F_t=-\mu_dF_n\tanh(v_d/v_c)t-\mu_rF_n\tanh(v_r/v_c)r
$$

この形は分離とすべりを扱え、正本モデル`ball_balancing_omni3_multibody_lqi_rev2.slx`のcustom contact物理モデルに対応する。輪–球接触の接線摩擦は異方性（駆動方向とローラー方向で別係数）とし、球–床接触には転がり抵抗を別途印加する。

## 7. 出力方程式

出力選択行列を使う場合、位置と車輪角は状態から直接選び、姿勢だけ非線形変換する。

$$
\boxed{y=h_y(x)=
\begin{bmatrix}
p_K^W\\
\operatorname{rpy}_{ZYX}(q_{WK})\\
p_B^W\\
\operatorname{rpy}_{ZYX}(q_{WB})\\
\theta_w
\end{bmatrix}}
$$

したがって入力から出力への直接項はなく、$D=0$である。静止直立平衡点$(x_0,u_0=0)$の周りでは、拘束を消去した独立状態$\delta x_r$に対して、

$$
\delta\dot x_r=A\delta x_r+B\delta u,qquad
\delta y=C\delta x_r+D\delta u,qquad D=0
$$

を数値線形化で得る。接触を含むため、$A,B$は手計算の固定行列ではなく、整合平衡点と採用する接触モデルから生成する。

## 8. パラメーター

### 8.1 既存値

| 記号 | 値 | 単位 | 実装 |
|---|---:|---|---|
| $g$ | 9.80665 | m/s$^2$ | `p.gravity` |
| $R_b$ | 0.050 | m | `p.ball.radius` |
| $m_b$ | 0.285 | kg | `p.ball.mass` |
| $I_b$ | $4.75\times10^{-4}I_3$ | kg m$^2$ | `p.ball.inertia` |
| $R_w$ | 0.024 | m | `p.wheel.radius` |
| $m_w$ | 0.032 | kg/輪 | `p.wheel.mass`（goBILDA 48 mm） |
| $\beta_i$ | $[0,120,240]$ | deg | `p.wheel.azimuth` |
| $\lambda$ | 33 | deg | `p.wheel.contactLatitude`（0.576 rad） |
| $\delta_p$ | $4.0\times10^{-4}$ | m | `p.wheel.contactPreload` |
| $m_R$ | 1.866（ペイロード0.5 kg込み、実機部1.366） | kg | `p.rover.mass` |
| $h_{BK}$ | 0.143 | m | 機体原点の幾何高さ、`p.rover.centerAboveBall` |
| $h_{COM}$ | 0.1812 | m | ローバー合成重心高（Fusion実測 MotorMount_v12 反映後）、`p.rover.comAboveBall` |
| $I_B$ | $\operatorname{diag}(0.00729,0.00729,0.00312)$ | kg m$^2$ | `p.rover.bodyInertia` |
| $R_c$ | 0.080 | m | `p.rover.chassisRadius` |
| $k_{n,w},c_{n,w}$ | $1.36\times10^4,170$ | N/m, N/(m/s) | `p.contact.wheelBall.*`（膜$3\times10^4$とローラー層$2.5\times10^4$の直列） |
| $k_{n,g},c_{n,g}$ | $3.0\times10^4,500$ | N/m, N/(m/s) | `p.contact.ballGround.*` |
| $\mu_d,\mu_r$（輪–球） | 2.0, 0.02 | - | `p.contact.wheelBall.*Friction` |
| $\mu_{s},\mu_{d}$（球–床） | 1.2, 1.2 | - | `p.contact.ballGround.*Friction` |
| $r_f$（転がり抵抗） | 0.008（許容帯0.005–0.015） | - | `p.contact.ballGround.rollingFriction` |
| $v_c$ | 0.005 | m/s | `p.contact.*.criticalVelocity` |
| 輪軸摩擦 | Coulomb 0.0199・breakaway 0.0239 | N·m | `ballbotShaftFrictionTorque`（augment追加） |

状態方程式は実行モデルおよび実機CAD測定値に合わせて$\lambda=33$ degを採用する（案B内向きモーター格納の幾何学的成立限界）。機体質量分布はFusion 360 実測の MotorMount_v12 部品表へ整合済みである。

### 8.2 実機化時に同定・更新する値

| パラメーター | 必要性 | 推奨取得方法 |
|---|---|---|
| ボール実測質量・慣性・反発 | プラント真値の更新 | 実球の計量・落下試験 |
| 輪–球・球–床摩擦係数 | トラクション限界の精度 | 実機の傾斜保持・滑り試験 |
| ゴムローラー層剛性 | 接触剛性直列合成の実測裏付け | ベンチ圧縮試験（現値は推定） |
| 駆動系デッドゾーン | FR-16補償値の個体差 | 実機デューティ掃引同定 |
| IMUノイズ・バイアス・温度特性 | 推定器共分散・学習閾値 | 実機IMU静止ログ・温度試験 |

これらはSimscape Multibodyモデルではパラメーター構造体 `ballbotParams`（`ballbotRev1Parameters`生成）へ一元化されている。

## 9. モデル利用上の前提

- 3輪すべてとボール–床が接触する運転領域では、無すべりDAEを使用できる。
- 転倒、接触分離、駆動方向すべりを含める場合は、ペナルティ接触形を使用する。
- 入力を車輪トルクにすると、駆動系の飽和・不感帯評価にはduty→トルクの電気系モデル（FIT0521＋MDD3A）を併せて使う。理想速度源モデルではトルクは拘束反力$\tau_a$になるため、実サーボの飽和評価には速度制御器とモータートルクを追加する。
- 絶対位置と絶対ヨーは、外部位置・方位センサーなしでは観測できない。ただしプラント真値としては状態および出力に保持できる。

## 10. 関連文書

- [制御・状態推定理論](ball_balancing_omnirover3wd-control-estimation-theory.md)
- [制御・推定アーキテクチャ](ball_balancing_omnirover3wd-architecture.md)
- [システム仕様](ball_balancing_omnirover3wd-system.md)
- [検証計画・ロバスト性受け入れ試験](ball_balancing_omnirover3wd-test-plan.md)
- [仕様変更記録](ball_balancing_omnirover3wd-change-history.md)
