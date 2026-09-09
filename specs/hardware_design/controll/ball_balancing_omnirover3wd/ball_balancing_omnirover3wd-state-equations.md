# 玉乗り3WDオムニローバー状態方程式

## 1. 対象と結論

車輪角速度を入力とし、ボールとローバーの位置・姿勢および車輪回転変位を出力とするプラントを、非線形記述系（DAE）として定義する。

車輪角速度3量だけでは、ボール角速度3量とローバー角速度3量を運動学だけから一意に分離できない。このため、絶対運動を表すプラントには質量・慣性・重力・接触反力を含む運動方程式が必要である。車輪速度から姿勢までを直接写像する陽な運動学モデルは、この機構の自由度を失うため採用しない。

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
| $u=\omega_w$ | 入力となる3輪の回転速度 | rad/s |

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
u=\dot\theta_w=
\begin{bmatrix}\omega_{w,1}&\omega_{w,2}&\omega_{w,3}\end{bmatrix}^T
\in\mathbb{R}^{3}
$$

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
\dot\theta_w=u
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

質量・慣性と接触反力を含むプラントは、次のDAEで表す。

$$
\boxed{
\begin{aligned}
\dot q &= T(q)\nu\\
M(q)\dot\nu+h(q,\nu) &= S_w^T\tau_a+J_n(q)^T\lambda_n+J_t(q)^T\lambda_t\\
0&=\Phi_n(q)\\
0&=J_t(q)\nu-D_wu\\
0&=S_w\nu-u
\end{aligned}}
$$

| 項 | 内容 |
|---|---|
| $M(q)$ | ボール、機体、サーボ、車輪を含む一般化質量行列 |
| $h(q,\nu)$ | コリオリ・遠心項、重力、受動摩擦 |
| $S_w$ | 一般化速度から車輪軸速度を選ぶ行列 |
| $\tau_a$ | 理想速度源が拘束維持のため発生する代数的な車輪軸トルク |
| $J_n^T\lambda_n$ | 床1点と車輪3点の法線反力 |
| $J_t^T\lambda_t$ | 無すべり時の接線反力 |

$J_t\nu-D_wu=0$の車輪3行と$S_w\nu-u=0$は、一般化速度の取り方に応じて同じ車輪速度条件を二重に課さない。車輪角を独立一般化座標に含む本定義では、$S_w\nu-u=0$を車輪軸拘束、$J_t\nu-D_wu=0$を接触点速度拘束として両方用いる。

初期値は$\Phi_n(q_0)=0$と速度拘束を満たす整合初期値でなければならない。速度指令が不連続な場合、理想速度源はインパルス状トルクを要求するため、実装では車輪速度指令に一次遅れまたは加速度制限を加える。

### 6.1 ペナルティ接触を使う既存Simscapeモデル

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

この形は分離とすべりを扱え、`ball_balancing_omni3_multibody_lqi_custom_contact.slx`へ次工程で実装するcustom contact物理モデルに対応する。

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
| $m_w$ | 0.039 | kg/輪 | `p.wheel.mass` |
| $\beta_i$ | $[0,120,240]$ | deg | `p.wheel.azimuth` |
| $\lambda$ | 55 | deg | `p.wheel.contactLatitude` |
| $\delta_p$ | $5.0\times10^{-5}$ | m | `p.wheel.contactPreload` |
| $m_R$ | 1.085 | kg | `p.rover.mass` |
| $h_{BK}$ | 0.125 | m | 機体原点の幾何高さ、`p.rover.centerAboveBall` |
| $h_{COM}$ | 0.2010 | m | 上部ロッド込み合成重心高、`p.rover.comAboveBall` |
| $k_{n,w},c_{n,w}$ | $2.0\times10^5,250$ | N/m, N/(m/s) | `p.contact.wheelBall.*` |
| $k_{n,g},c_{n,g}$ | $3.0\times10^5,180$ | N/m, N/(m/s) | `p.contact.ballGround.*` |
| $\mu_d,\mu_r$ | 0.75, 0.02 | - | `p.contact.wheelBall.*Friction` |
| $v_c$ | 0.005 | m/s | `p.contact.wheelBall.criticalVelocity` |

状態方程式は実行モデルに合わせて$\lambda=55$ degを採用する。既存の制御・推定理論文書にある45 degとは不一致であり、幾何を固定する際に統一が必要である。

### 8.2 独立実装前に追加・同定する値

| パラメーター | 必要性 | 推奨取得方法 |
|---|---|---|
| ローバー重心位置$r_{G_B/B}^B$ | 重力モーメントと質量行列 | 組立CADまたは吊り下げ実測 |
| ローバー慣性テンソル$I_B$ | ロール・ピッチ・ヨー運動 | 組立CADから抽出し振り子試験で確認 |
| 各車輪の重心・慣性$I_{w,i}$ | 速度源の反力と高速運動 | CADまたはメーカー値 |
| $r_{F_i/B}^B$と車輪軸方向 | 接触ヤコビアン | 組立CADから抽出 |
| サーボ速度ループ帯域・加速度上限 | 理想速度源の非現実的トルクを回避 | 実機ステップ応答 |
| 転がり抵抗、粘性摩擦 | 定常速度と減衰 | 惰行試験 |

これらは既存Simscape Multibodyモデルでは各Solid、Rigid Transform、Jointに分散している。制御設計用の独立した$M(q)$を生成する場合は、`ballbotParameters.m`へ一元化してから線形化する。

## 9. モデル利用上の前提

- 3輪すべてとボール–床が接触する運転領域では、無すべりDAEを使用できる。
- 転倒、接触分離、駆動方向すべりを含める場合は、ペナルティ接触形を使用する。
- 入力を車輪速度にすると、サーボトルクは状態入力ではなく拘束反力$\tau_a$になる。実サーボの飽和評価には、速度制御器とモータートルクを追加する。
- 絶対位置と絶対ヨーは、外部位置・方位センサーなしでは観測できない。ただしプラント真値としては状態および出力に保持できる。

## 10. 関連文書

- [制御・状態推定理論](ball_balancing_omnirover3wd-control-estimation-theory.md)
- [制御・推定アーキテクチャ](ball_balancing_omnirover3wd-architecture.md)
- [システム仕様](ball_balancing_omnirover3wd-system.md)
