"""FIT0521モーター固定パーツ + フレーム1層プレート + ボールキャスターの生成・検証スクリプト。

FIT0521(φ24.4 x 52 mm、4 mm Dシャフト)はホイール軸と同軸だが、車体中心側
(+axle側)に置くと3本のモーター軸が中心軸上で交差し互いに衝突するため、
ホイール外側(-axle側)へ外向き下方に配置する。モーター固定パーツは
ギアボックス周辺(前面から約22 mmの筒部)だけを覆うクランプ式サドルとし、
フレーム1層へはプレート下面の水平延長パッドを介してM4ネジ+ナットで固定する。

- フレーム層への固定穴は、周囲半径6 mm以上に他形状のない板面上に配置
- 上記以外のねじ穴はM3熱圧入インサートナット前提のポケット
- 上記以外の面は可能な限り連続した1面で構成
- ボールキャスター(汎用ボールキャスター仮モデル)は玉乗り球の緯度+30°に接触し、
  ホイール方位の中間(β=60/180/300°)に配置
- 座標原点は100 mm球中心、+Z上向き(=既存レイアウトと同一規約)

実行例:
    uv run --with build123d --with matplotlib python fit0521_motor_mount.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from build123d import (
    Align,
    Box,
    Compound,
    Cylinder,
    Face,
    Plane,
    Pos,
    Rot,
    Solid,
    Sphere,
    Vector,
    Wire,
    export_step,
    export_stl,
    import_step,
)


HERE = Path(__file__).resolve().parent
MECHANISM_ROOT = HERE.parents[1]
WHEEL_STEP = (
    MECHANISM_ROOT
    / "omnirover3wd_reference"
    / "omniwheel_14108"
    / "48MM-OMNI-WHEEL-for-NXT-and-servo-motor 14108"
    / "48MM-OMNI-WHEEL-for-NXT-and-servo-motor.stp"
)

# --- 球・ホイール・配置(制御仕様と一致) ---
BALL_RADIUS = 50.0
WHEEL_RADIUS = 24.0
WHEEL_WIDTH = 25.1
CONTACT_LATITUDE = 55.0
AZIMUTHS = (0.0, 120.0, 240.0)
WHEEL_CENTRE_LOCAL = Vector(-17.0845, 24.8661, 0.0)

# --- FIT0521 寸法(公称/実測待ち) ---
MOTOR_DIAMETER = 24.4          # メーカー公称
MOTOR_LENGTH = 52.0            # メーカー公称
MOTOR_SHAFT_DIAMETER = 4.0     # ディーラ寸法(Dカット)
MOTOR_SHAFT_LENGTH = 12.0      # ディーラ寸法
ENCODER_BOSS_DIAMETER = 20.0   # 仮: エンコーダー・コネクタ突起の包絡
ENCODER_BOSS_LENGTH = 6.0      # 仮
FACE_CLEARANCE = 4.0           # モーター前面 - ハブアダプタフランジ間の隙間

# --- ハブアダプタ(12 mm六角穴 -> 4 mm Dシャフト、別部品・寸法仮) ---
ADAPTER_HEX_FLATS = 11.8       # 12 mm六角穴のクリアランス込み
ADAPTER_HEX_LENGTH = 10.0
ADAPTER_FLANGE_DIAMETER = 20.0
ADAPTER_FLANGE_THICKNESS = 2.5

# --- フレーム1層プレート(nexus16007_motor_layer互換・本ビルド用に更新) ---
PLATE_OUTER_DIAMETER = 140.0
PLATE_INNER_DIAMETER = 96.0
PLATE_THICKNESS = 6.0
PLATE_BOTTOM_Z = 30.0                      # 球中心基準(=床から80 mm)
STACK_HOLE_COUNT = 6
STACK_HOLE_DIAMETER = 4.5
STACK_HOLE_PCD = 128.0
STACK_HOLE_PHASE_DEG = 30.0
MOTOR_NOTCH_RADIUS_START = 55.0            # 外縁のモーターコリドー切欠き(モーター下面はr~56でプレート上面を割る)
MOTOR_NOTCH_TANGENTIAL_HALF = 19.0
FLANGE_BOLT_RADIUS = 56.0                  # サドル水平パッドのM4固定位置
FLANGE_BOLT_TANGENTIAL = 24.0
FRAME_BOLT_CLEARANCE = 4.5                 # M4すきま穴

# --- サドル(モーター固定パーツ、ギアボックス筒部のみを覆う) ---
CRADLE_WIDTH = 44.0                        # 接線方向幅
CRADLE_GROOVE_RADIUS = 12.6                # モーター胴体より0.4大きい半円溝
CRADLE_LIP_Z = 2.0                         # 溝のリップ高さ(軸心より上)
CRADLE_Y_FRONT = -2.0                      # クラドル前端(モーター前面=0の+y側)
CRADLE_Y_REAR = -24.0                      # クラドル後端(ギアボックス約22 mmを覆う)
CRADLE_BOTTOM_Z = -14.6                    # 溝底+2 mm壁(モーター中心より14.6下)
CORRIDOR_HALF_WIDTH = 15.0                 # プレートを通るモーター/サドル脚のコリドー半幅
PAD_THICKNESS = 4.0                        # プレート下面の水平延長パッド厚
PAD_TOP_Z = PLATE_BOTTOM_Z                 # パッド上面=プレート下面
PAD_RADIAL = (49.0, 63.0)                  # 穴中心r=56から半径6 mm以上に縁のない領域
PAD_TANGENTIAL_HALF = 30.0                 # 穴中心t=±24から半径6 mm以上に縁のない領域
INSERT_POCKET_DIAMETER = 4.6               # M3熱圧入インサート前提
INSERT_POCKET_DEPTH = 7.0
INSERT_POCKET_X = 17.5
INSERT_POCKET_Y = (-7.0, -17.0)
CAP_HEIGHT = 16.0                          # キャップ天面(溝頂部より上3.3 mm壁)
CAP_GROOVE_RADIUS = 12.7
CAP_SCREW_DIAMETER = 3.2                   # M3すきま穴

# --- ボールキャスター(汎用ボールキャスター仮モデル、実物で確定) ---
CASTER_LATITUDE = 30.0                     # 球面の接触緯度(赤道=0、ホイールと同じ半球側)
CASTER_AZIMUTHS = (60.0, 180.0, 300.0)     # ホイール方位の中間
CASTER_BALL_RADIUS = 5.0                   # 仮: キャスターボールØ10
CASTER_BODY_DIAMETER = 13.0                # 仮: キャスター本体Ø13
CASTER_BODY_LENGTH = 14.0                  # 仮: ソケットに入る胴体長
CASTER_BORE_CLEARANCE = 0.6                # ソケット内径のクリアランス径
CASTER_BALL_CLEARANCE = 0.6
CASTER_HOLDER_HALF_WIDTH = 13.0            # ホルダー断面の接線方向半幅
CASTER_HOLDER_FACE_S = 17.0                # ボール中心からホルダー端面までの距離(n_c方向)
CASTER_HOLDER_REAR_S = -7.0                # ボール中心より開口側への張り出し
CASTER_INSERT_TANGENTIAL = 8.0             # フランジ固定用インサートポケットの位置
CASTER_PAD_TANGENTIAL_HALF = 19.0          # ホルダー耳(プレート上面パッド)の半幅
CASTER_PAD_RADIAL = (56.0, 70.0)
CASTER_PAD_BOTTOM_Z = PLATE_BOTTOM_Z + PLATE_THICKNESS  # プレート上面
CASTER_PAD_HEIGHT = 3.5
CASTER_BOLT_RADIUS = 62.0                  # ホルダー耳のM4固定位置(周囲6 mmクリア)
CASTER_BOLT_TANGENTIAL = 12.0
CASTER_NOTCH_RADIUS = (44.0, 62.0)         # キャスター通過用の内縁ベイ切欠き
CASTER_NOTCH_TANGENTIAL_HALF = 8.0


def _frame(azimuth: float):
    """azimuthの車輪について (radial, tangent, normal, axle) を返す。"""
    b = math.radians(azimuth)
    lam = math.radians(CONTACT_LATITUDE)
    radial = Vector(math.cos(b), math.sin(b), 0.0)
    tangent = Vector(-math.sin(b), math.cos(b), 0.0)
    normal = Vector(math.cos(lam) * radial.X, math.cos(lam) * radial.Y, math.sin(lam))
    axle = Vector(-math.sin(lam) * radial.X, -math.sin(lam) * radial.Y, math.cos(lam))
    return radial, tangent, normal, axle


def _caster_frame(azimuth: float):
    """azimuth方位のキャスターについて (radial, tangent, normal) を返す。"""
    b = math.radians(azimuth)
    lam = math.radians(CASTER_LATITUDE)
    radial = Vector(math.cos(b), math.sin(b), 0.0)
    tangent = Vector(-math.sin(b), math.cos(b), 0.0)
    normal = Vector(math.cos(lam) * radial.X, math.cos(lam) * radial.Y, math.sin(lam))
    return radial, tangent, normal


def _polar(radius: float, tangent_offset: float, azimuth: float) -> Vector:
    """azimuth方位の (半径方向radius, 接線方向tangent_offset) の点。"""
    radial, tangent, _, _ = _frame(azimuth)
    return radial * radius + tangent * tangent_offset


def _through_hole(radius: float, tangent_offset: float, azimuth: float,
                  diameter: float, z_min: float, z_max: float) -> Solid:
    p = _polar(radius, tangent_offset, azimuth)
    return Pos(p.X, p.Y, z_min) * Cylinder(
        diameter / 2, z_max - z_min,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )


def make_plate() -> Solid:
    plate = Cylinder(
        PLATE_OUTER_DIAMETER / 2,
        PLATE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    plate = Pos(0, 0, PLATE_BOTTOM_Z) * plate

    opening = Pos(0, 0, PLATE_BOTTOM_Z - 1) * Cylinder(
        PLATE_INNER_DIAMETER / 2, PLATE_THICKNESS + 2,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    plate = plate - opening

    for index in range(STACK_HOLE_COUNT):
        angle = math.radians(STACK_HOLE_PHASE_DEG + 360.0 * index / STACK_HOLE_COUNT)
        x, y = STACK_HOLE_PCD / 2 * math.cos(angle), STACK_HOLE_PCD / 2 * math.sin(angle)
        plate = plate - Pos(x, y, PLATE_BOTTOM_Z - 1) * Cylinder(
            STACK_HOLE_DIAMETER / 2, PLATE_THICKNESS + 2,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )

    for azimuth in AZIMUTHS:
        # 外縁のモーターコリドー切欠き(半径55 mmから外縁まで、接線方向に幅38)
        notch_centre_r = (MOTOR_NOTCH_RADIUS_START + PLATE_OUTER_DIAMETER / 2) / 2
        notch = Rot(0, 0, azimuth) * Pos(
            notch_centre_r, 0, PLATE_BOTTOM_Z - 1
        ) * Box(
            PLATE_OUTER_DIAMETER / 2 - MOTOR_NOTCH_RADIUS_START + 2,
            2 * MOTOR_NOTCH_TANGENTIAL_HALF,
            PLATE_THICKNESS + 2,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        plate = plate - notch
        for sign in (-1.0, 1.0):
            plate = plate - _through_hole(
                FLANGE_BOLT_RADIUS, sign * FLANGE_BOLT_TANGENTIAL, azimuth,
                FRAME_BOLT_CLEARANCE, PLATE_BOTTOM_Z - 1, PLATE_BOTTOM_Z + PLATE_THICKNESS + 1)

    for azimuth in CASTER_AZIMUTHS:
        # 内縁のキャスター通過ベイ(中央開口へつながる)
        bay = Rot(0, 0, azimuth) * Pos(
            (CASTER_NOTCH_RADIUS[0] + CASTER_NOTCH_RADIUS[1]) / 2,
            0, PLATE_BOTTOM_Z - 1,
        ) * Box(
            CASTER_NOTCH_RADIUS[1] - CASTER_NOTCH_RADIUS[0],
            2 * CASTER_NOTCH_TANGENTIAL_HALF,
            PLATE_THICKNESS + 2,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        plate = plate - bay
        for sign in (-1.0, 1.0):
            plate = plate - _through_hole(
                CASTER_BOLT_RADIUS, sign * CASTER_BOLT_TANGENTIAL, azimuth,
                FRAME_BOLT_CLEARANCE, PLATE_BOTTOM_Z - 1, PLATE_BOTTOM_Z + PLATE_THICKNESS + 1)

    plate.label = "fit0521_motor_layer_plate"
    return plate


def _mount_plane(azimuth: float) -> Plane:
    """モーター前面中心を原点とするローカル平面。

    x=接線、y=車軸上り方向(+axle=車輪側)、z=溝開放方向(+normal)。
    """
    _, tangent, normal, axle = _frame(azimuth)
    wheel_centre = normal * (BALL_RADIUS + WHEEL_RADIUS)
    face_centre = wheel_centre - axle * (WHEEL_WIDTH / 2 + ADAPTER_FLANGE_THICKNESS + FACE_CLEARANCE)
    return Plane(origin=face_centre, x_dir=tangent, z_dir=normal)


def make_cradle(azimuth: float, plate: Solid) -> Solid:
    plane = _mount_plane(azimuth)
    # ローカル箱: x=接線、y=車軸方向(モーター本体は-y側)、z=溝開放方向
    block = Box(
        CRADLE_WIDTH,
        CRADLE_Y_FRONT - CRADLE_Y_REAR,
        CRADLE_LIP_Z - CRADLE_BOTTOM_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    block = Pos(0, (CRADLE_Y_REAR + CRADLE_Y_FRONT) / 2, CRADLE_BOTTOM_Z) * block
    block = plane.location * block

    # モーター胴体の半円溝(軸=y方向)
    groove = plane.location * Pos(0, (CRADLE_Y_REAR + CRADLE_Y_FRONT) / 2 - 5, 0) * Rot(90, 0, 0) * Cylinder(
        CRADLE_GROOVE_RADIUS, CRADLE_Y_FRONT - CRADLE_Y_REAR + 10.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    block = block - groove

    # インサートナット座(M3熱圧入前提、リップ上面から下向き)
    for x_sign in (-1.0, 1.0):
        for y_pos in INSERT_POCKET_Y:
            pocket = plane.location * Pos(
                x_sign * INSERT_POCKET_X, y_pos, CRADLE_LIP_Z - INSERT_POCKET_DEPTH
            ) * Cylinder(
                INSERT_POCKET_DIAMETER / 2, INSERT_POCKET_DEPTH + 0.5,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
            block = block - pocket

    # プレート上面より下でモーター通過コリドー(|t|<15)の外に残る細片を除去
    trim_zone = Pos(0, 0, -50) * Cylinder(
        PLATE_OUTER_DIAMETER / 2, PLATE_BOTTOM_Z + PLATE_THICKNESS + 50.01,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    corridor = Rot(0, 0, azimuth) * Pos(
        0, -CORRIDOR_HALF_WIDTH, PLATE_BOTTOM_Z - PLATE_THICKNESS
    ) * Box(
        2 * PLATE_OUTER_DIAMETER,
        2 * CORRIDOR_HALF_WIDTH,
        2 * PLATE_THICKNESS,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )
    block = block - (trim_zone - corridor)

    # パッド下面に合わせて脚の下端を揃える
    below_pad = Pos(0, 0, -50) * Cylinder(
        PLATE_OUTER_DIAMETER / 2, PAD_TOP_Z - PAD_THICKNESS + 50.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    block = block - plate
    block = block - below_pad

    # プレート下面の水平延長パッド(連続した1面、M4ネジ+ナットで締結)
    pad = Rot(0, 0, azimuth) * Pos(
        PAD_RADIAL[0], -PAD_TANGENTIAL_HALF, PAD_TOP_Z - PAD_THICKNESS
    ) * Box(
        PAD_RADIAL[1] - PAD_RADIAL[0],
        2 * PAD_TANGENTIAL_HALF,
        PAD_THICKNESS,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )
    block = block + pad

    # フランジ固定用M4すきま穴(パッド貫通、垂直)
    for sign in (-1.0, 1.0):
        block = block - _through_hole(
            FLANGE_BOLT_RADIUS, sign * FLANGE_BOLT_TANGENTIAL, azimuth,
            FRAME_BOLT_CLEARANCE, PAD_TOP_Z - PAD_THICKNESS - 1, PAD_TOP_Z + 1)

    block.label = f"fit0521_motor_mount_cradle:{int(azimuth)}deg"
    return block


def make_cap(azimuth: float) -> Solid:
    plane = _mount_plane(azimuth)
    cap = Box(
        CRADLE_WIDTH,
        CRADLE_Y_FRONT - CRADLE_Y_REAR,
        CAP_HEIGHT - CRADLE_LIP_Z,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    cap = Pos(0, (CRADLE_Y_REAR + CRADLE_Y_FRONT) / 2, CRADLE_LIP_Z) * cap
    groove = Pos(0, (CRADLE_Y_REAR + CRADLE_Y_FRONT) / 2 - 5, 0) * Rot(90, 0, 0) * Cylinder(
        CAP_GROOVE_RADIUS, CRADLE_Y_FRONT - CRADLE_Y_REAR + 10.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    cap = cap - groove
    for x_sign in (-1.0, 1.0):
        for y_pos in INSERT_POCKET_Y:
            cap = cap - Pos(x_sign * INSERT_POCKET_X, y_pos, CRADLE_LIP_Z - 0.5) * Cylinder(
                CAP_SCREW_DIAMETER / 2, CAP_HEIGHT + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            )
    cap = plane.location * cap
    cap.label = f"fit0521_motor_mount_cap:{int(azimuth)}deg"
    return cap


def _caster_plane(azimuth: float) -> Plane:
    """キャスターボール中心を原点とするローカル平面。

    z=球外向き法線(緯度30°方向)、x=接線。キャスター本体は+z側、
    ボールは-z側に張り出して球に接する。
    """
    _, tangent, normal = _caster_frame(azimuth)
    centre = normal * (BALL_RADIUS + CASTER_BALL_RADIUS)
    return Plane(origin=centre, x_dir=tangent, z_dir=normal)


def make_caster_holder(azimuth: float, plate: Solid) -> Solid:
    """球緯度+30°に接するボールキャスターのソケットホルダー。"""
    plane = _caster_plane(azimuth)
    holder = Box(
        2 * CASTER_HOLDER_HALF_WIDTH,
        2 * CASTER_HOLDER_HALF_WIDTH,
        CASTER_HOLDER_FACE_S - CASTER_HOLDER_REAR_S,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    holder = Pos(0, 0, CASTER_HOLDER_REAR_S) * holder

    # キャスターボールと本体のソケット(軸=球法線方向)
    ball_pocket = Pos(0, 0, -CASTER_BALL_RADIUS - CASTER_BALL_CLEARANCE) * Cylinder(
        (CASTER_BALL_RADIUS * 2 + CASTER_BALL_CLEARANCE) / 2,
        CASTER_BALL_RADIUS + CASTER_BALL_CLEARANCE + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    body_bore = Pos(0, 0, 0) * Cylinder(
        (CASTER_BODY_DIAMETER + CASTER_BORE_CLEARANCE) / 2,
        CASTER_BODY_LENGTH + 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    holder = holder - ball_pocket - body_bore

    # キャスターフランジ固定用のインサートナット座(M3、端面から内向き)
    for x_sign in (-1.0, 1.0):
        pocket = Pos(
            x_sign * CASTER_INSERT_TANGENTIAL, 0,
            CASTER_HOLDER_FACE_S - INSERT_POCKET_DEPTH,
        ) * Cylinder(
            INSERT_POCKET_DIAMETER / 2, INSERT_POCKET_DEPTH + 0.5,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
        holder = holder - pocket

    holder = plane.location * holder

    # プレート上面の固定耳(連続した1面、M4ネジ+ナットで締結)
    pad = Rot(0, 0, azimuth) * Pos(
        CASTER_PAD_RADIAL[0], -CASTER_PAD_TANGENTIAL_HALF, CASTER_PAD_BOTTOM_Z
    ) * Box(
        CASTER_PAD_RADIAL[1] - CASTER_PAD_RADIAL[0],
        2 * CASTER_PAD_TANGENTIAL_HALF,
        CASTER_PAD_HEIGHT,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )
    holder = holder + pad

    # プレートとの交差部はプレート形状に倣わせる
    holder = holder - plate

    # 耳のM4すきま穴
    for sign in (-1.0, 1.0):
        holder = holder - _through_hole(
            CASTER_BOLT_RADIUS, sign * CASTER_BOLT_TANGENTIAL, azimuth,
            FRAME_BOLT_CLEARANCE, CASTER_PAD_BOTTOM_Z - 1,
            CASTER_PAD_BOTTOM_Z + CASTER_PAD_HEIGHT + 1)

    holder.label = f"ball_caster_holder:{int(azimuth)}deg"
    return holder


def make_caster_unit(azimuth: float) -> Compound:
    """汎用ボールキャスターの仮モデル(ボール+胴体+フランジ)。"""
    plane = _caster_plane(azimuth)
    ball = plane.location * Pos(0, 0, 0) * Sphere(CASTER_BALL_RADIUS)
    body = plane.location * Pos(0, 0, 0) * Cylinder(
        CASTER_BODY_DIAMETER / 2, CASTER_BODY_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    flange = plane.location * Pos(0, 0, CASTER_BODY_LENGTH) * Cylinder(
        12.0, 2.0,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    unit = Compound(children=[ball, body, flange])
    unit.label = f"ball_caster_unit:{int(azimuth)}deg"
    return unit


def make_motor(azimuth: float) -> Compound:
    plane = _mount_plane(azimuth)
    # 本体: 前面から-y方向へ52 mm
    body = plane.location * Pos(0, -MOTOR_LENGTH / 2, 0) * Rot(90, 0, 0) * Cylinder(
        MOTOR_DIAMETER / 2, MOTOR_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    # 出力軸: 前面から+y方向へ12 mm
    shaft = plane.location * Pos(0, MOTOR_SHAFT_LENGTH / 2, 0) * Rot(90, 0, 0) * Cylinder(
        MOTOR_SHAFT_DIAMETER / 2, MOTOR_SHAFT_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    # 後端のエンコーダー・コネクタ包絡突起
    boss = plane.location * Pos(0, -MOTOR_LENGTH - ENCODER_BOSS_LENGTH / 2, 0) * Rot(90, 0, 0) * Cylinder(
        ENCODER_BOSS_DIAMETER / 2, ENCODER_BOSS_LENGTH,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    motor = Compound(children=[body, shaft, boss])
    motor.label = f"fit0521_motor:{int(azimuth)}deg"
    return motor


def make_adapter(azimuth: float) -> Compound:
    """12 mm六角ハブ -> 4 mm Dシャフトの変換アダプタ(簡略モデル)。"""
    _, tangent, normal, axle = _frame(azimuth)
    wheel_centre = normal * (BALL_RADIUS + WHEEL_RADIUS)
    # -axle側の車輪端面中心
    outer_face = wheel_centre - axle * (WHEEL_WIDTH / 2)
    hex_prism = Pos(0, 0, -ADAPTER_HEX_LENGTH / 2) * extrude_hex(ADAPTER_HEX_FLATS, ADAPTER_HEX_LENGTH)
    flange = Pos(0, 0, -ADAPTER_HEX_LENGTH - ADAPTER_FLANGE_THICKNESS / 2) * Cylinder(
        ADAPTER_FLANGE_DIAMETER / 2, ADAPTER_FLANGE_THICKNESS,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    adapter = hex_prism + flange
    adapter = Plane(origin=outer_face, z_dir=-axle).location * adapter
    adapter.label = f"hub_adapter_hex12_d4:{int(azimuth)}deg"
    return adapter


def extrude_hex(flats: float, length: float) -> Solid:
    r = flats / math.sqrt(3.0)  # 面間寸法 -> 外接半径
    pts = [
        Vector(r * math.cos(math.radians(60 * i + 30)), r * math.sin(math.radians(60 * i + 30)), 0)
        for i in range(6)
    ]
    wire = Wire.make_polygon(pts, close=True)
    return Solid.extrude(Face(wire), Vector(0, 0, 1))


def _place_wheel(source, azimuth: float):
    _, tangent, normal, axle = _frame(azimuth)
    centre = normal * (BALL_RADIUS + WHEEL_RADIUS)
    local = Pos(-WHEEL_CENTRE_LOCAL.X, -WHEEL_CENTRE_LOCAL.Y, 0) * source
    wheel = Plane(origin=centre, x_dir=normal, z_dir=-tangent).location * local
    wheel.label = f"omniwheel_14108:{int(azimuth)}deg"
    return wheel, centre, axle


def make_assembly(plate: Solid, cradles, caps, holders, casters) -> Compound:
    wheel_source = import_step(WHEEL_STEP)
    parts = [plate]
    ball = Sphere(BALL_RADIUS)
    ball.label = "rigid_ball_100mm"
    parts.append(ball)
    for azimuth, cradle, cap in zip(AZIMUTHS, cradles, caps):
        wheel, _, _ = _place_wheel(wheel_source, azimuth)
        parts.extend([wheel, make_motor(azimuth), make_adapter(azimuth), cradle, cap])
    parts.extend(holders)
    parts.extend(casters)
    assembly = Compound(children=parts)
    assembly.label = "fit0521_drive_layout"
    return assembly


def _triangles(shape):
    vertices, faces = shape.tessellate(0.35)
    xyz = [(v.X, v.Y, v.Z) for v in vertices]
    return [[xyz[i] for i in face] for face in faces]


def render(shape, path: Path, elev: float, azim: float, title: str) -> None:
    fig = plt.figure(figsize=(8, 8), dpi=160)
    ax = fig.add_subplot(111, projection="3d")
    tris = _triangles(shape)
    ax.add_collection3d(Poly3DCollection(tris, facecolor="#6aaed6", edgecolor="#234", linewidth=0.08))
    bb = shape.bounding_box()
    xs = [bb.min.X, bb.max.X]; ys = [bb.min.Y, bb.max.Y]; zs = [bb.min.Z, bb.max.Z]
    span = max(xs[1]-xs[0], ys[1]-ys[0], zs[1]-zs[0]) / 2
    cx = sum(xs)/2; cy = sum(ys)/2; cz = sum(zs)/2
    ax.set_xlim(cx-span, cx+span); ax.set_ylim(cy-span, cy+span); ax.set_zlim(cz-span, cz+span)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev=elev, azim=azim)
    ax.set_title(title); ax.set_xlabel("X [mm]"); ax.set_ylabel("Y [mm]"); ax.set_zlabel("Z [mm]")
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def _overlap(a, b) -> float:
    try:
        r = a.intersect(b)
        return float(r.volume) if r is not None else 0.0
    except Exception:
        return -1.0


def _export(shape, filename: str, stl: bool = False) -> None:
    export_step(shape, str(HERE / filename))
    if stl:
        export_stl(
            shape,
            str(HERE / filename).replace(".step", ".stl"),
            tolerance=0.08,
            angular_tolerance=0.1,
        )


def main() -> None:
    # OCCのブール/Compound化で元形状が汚染されるのを避けるため、生成直後にエクスポートする
    plate = make_plate()
    _export(plate, "fit0521_motor_layer_plate.step", stl=True)

    cradle0 = make_cradle(0.0, plate)
    _export(cradle0, "fit0521_motor_mount_cradle.step", stl=True)
    cap0 = make_cap(0.0)
    _export(cap0, "fit0521_motor_mount_cap.step", stl=True)
    holder0 = make_caster_holder(CASTER_AZIMUTHS[0], plate)
    _export(holder0, "ball_caster_holder.step", stl=True)

    cradles = [cradle0] + [make_cradle(a, plate) for a in AZIMUTHS[1:]]
    caps = [cap0] + [make_cap(a) for a in AZIMUTHS[1:]]
    holders = [holder0] + [make_caster_holder(a, plate) for a in CASTER_AZIMUTHS[1:]]
    motors = [make_motor(a) for a in AZIMUTHS]
    casters = [make_caster_unit(a) for a in CASTER_AZIMUTHS]

    wheel_source = import_step(WHEEL_STEP)
    wheels = [_place_wheel(wheel_source, a)[0] for a in AZIMUTHS]
    ball = Sphere(BALL_RADIUS)
    assembly = make_assembly(plate, cradles, caps, holders, casters)
    _export(assembly, "fit0521_drive_layout.step")

    render(cradle0, HERE / "fit0521_motor_mount_cradle_iso.png", 24, -60, "Motor mount cradle - iso")
    render(cap0, HERE / "fit0521_motor_mount_cap_iso.png", 24, -60, "Motor mount cap - iso")
    render(cradle0 + cap0, HERE / "fit0521_motor_mount_iso.png", 28, -52, "Motor mount (cradle+cap) - iso")
    render(holder0, HERE / "ball_caster_holder_iso.png", 24, -60, "Ball caster holder - iso")
    render(plate, HERE / "fit0521_motor_layer_plate_top.png", 90, -90, "Motor layer plate - top")
    render(plate, HERE / "fit0521_motor_layer_plate_iso.png", 30, -60, "Motor layer plate - iso")
    render(assembly, HERE / "fit0521_drive_layout_iso.png", 24, -48, "FIT0521 drive layout - iso")
    render(assembly, HERE / "fit0521_drive_layout_bottom.png", -35, -48, "FIT0521 drive layout - underside")

    motor0 = motors[0]
    wheel0 = wheels[0]
    caster0 = casters[0]
    report = {
        "valid": (
            all(c.is_valid for c in cradles) and cap0.is_valid and plate.is_valid
            and all(h.is_valid for h in holders)
        ),
        "solids_count": {
            "cradle": [len(c.solids()) for c in cradles],
            "cap": [len(c.solids()) for c in caps],
            "caster_holder": [len(h.solids()) for h in holders],
        },
        "face_count": {
            "cradle": len(cradle0.faces()),
            "cap": len(cap0.faces()),
            "plate": len(plate.faces()),
            "caster_holder": len(holder0.faces()),
        },
        "arrangement": "motor on -axle (outward-down) side, coaxial direct drive; ball casters at latitude +30deg",
        "overlap_mm3": {
            "cradle_motor": _overlap(cradle0, motor0),
            "cradle_wheel": _overlap(cradle0, wheel0),
            "cradle_plate": _overlap(cradle0, plate),
            "cap_motor": _overlap(cap0, motor0),
            "motor_wheel": _overlap(motor0, wheel0),
            "motor_plate": _overlap(motor0, plate),
            "motor_ball": _overlap(motor0, ball),
            "motor1_motor2": _overlap(motors[0], motors[1]),
            "wheel_ball": _overlap(wheel0, ball),
            "holder_cradle": _overlap(holder0, cradle0),
            "holder_plate": _overlap(holder0, plate),
            "holder_wheel": _overlap(holder0, wheel0),
            "caster_ball": _overlap(caster0, ball),
            "caster_holder0_holder120": _overlap(holders[0], holders[1]),
            "caster_plate": _overlap(caster0, plate),
        },
        "clearance_mm": {
            "cradle_motor_surface": round(cradle0.distance(motor0), 3),
            "cap_motor_surface": round(cap0.distance(motor0), 3),
            "cap_cradle_interface": round(cap0.distance(cradle0), 3),
            "motor_plate_surface": round(motor0.distance(plate), 3),
            "motor_wheel_surface": round(motor0.distance(wheel0), 3),
            "motor1_motor2_surface": round(motors[0].distance(motors[1]), 3),
            "wheel_ball_surface": round(wheel0.distance(ball), 3),
            "caster_ball_contact": round(caster0.distance(ball), 3),
            "holder_cradle": round(holder0.distance(cradle0), 3),
            "holder_motor": round(holder0.distance(motor0), 3),
            "holder_wheel": round(holder0.distance(wheel0), 3),
            "holder_caster_unit": round(holder0.distance(caster0), 3),
        },
        "bolt_hole_clearance_mm": {
            "motor_pad": "hole r56/t±24 on pad r49..63/|t|<=30 -> >=6mm clear ring on pad",
            "caster_pad": "hole r62/t±12 on pad r56..70/|t|<=19 -> >=6mm clear ring on pad",
        },
        "assumptions": [
            "ボールキャスターは品番未定の汎用モデル(ボールØ10、本体Ø13x14+フランジ)。実物品番でソケット寸法を確定",
            "キャスター接触緯度+30°はホイールと同じ半球側(上側)の解釈。下半球(-30°)ならCASTER_LATITUDEを負にして再生成",
            "FIT0521前面M3タップPCD 20.2 mmは25GA370系流通ブラケットの仕様からの仮値。実機採寸で確定",
            "ハブアダプタ(12 mm六角 -> 4 mm Dシャフト)は仮モデル。実物の形状・締結で確定",
            "モーター軸方向の固定はクランプ摩擦。滑りが出たら前面M3タップ固定の面金具を追加",
            "エンコーダー・コネクタ突起はφ20 x 6 mmの仮包絡",
        ],
        "envelope_mm": {
            "assembly_bbox": {
                "x": [round(assembly.bounding_box().min.X, 2), round(assembly.bounding_box().max.X, 2)],
                "y": [round(assembly.bounding_box().min.Y, 2), round(assembly.bounding_box().max.Y, 2)],
                "z": [round(assembly.bounding_box().min.Z, 2), round(assembly.bounding_box().max.Z, 2)],
            },
        },
    }
    (HERE / "fit0521_mount_validation.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
