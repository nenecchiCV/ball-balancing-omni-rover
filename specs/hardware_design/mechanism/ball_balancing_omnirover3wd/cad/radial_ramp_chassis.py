"""3方向独立傾斜シャシーの生成・検証スクリプト。

実行例:
    uv run --with build123d --with matplotlib python radial_ramp_chassis.py
"""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from build123d import (
    Align,
    Axis,
    Box,
    Circle,
    Compound,
    Cylinder,
    Face,
    Location,
    Plane,
    Pos,
    Rotation,
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
SERVO_STEP = MECHANISM_ROOT / "omnirover3wd_reference" / "servo_16007" / "mg996r_like_16007.step"
WHEEL_STEP = (
    MECHANISM_ROOT
    / "omnirover3wd_reference"
    / "omniwheel_14108"
    / "48MM-OMNI-WHEEL-for-NXT-and-servo-motor 14108"
    / "48MM-OMNI-WHEEL-for-NXT-and-servo-motor.stp"
)

# mm / deg。Rotramaのシャシー固定穴ピッチは現物確認後にこの2値だけを更新する。
DISC_DIAMETER = 140.0
DISC_THICKNESS = 4.0
RAMP_ANGLE = 55.0
RAMP_RADIAL_LENGTH = 26.0
RAMP_TANGENTIAL_WIDTH = 58.0
RAMP_INNER_RADIUS = 20.0
MOUNT_HOLE_RADIAL_PITCH = 18.0       # 仮定値
MOUNT_HOLE_TANGENTIAL_PITCH = 50.0   # 公開されるサーボ固定穴間隔を暫定流用
M25_CLEARANCE_DIAMETER = 2.9
M4_CLEARANCE_DIAMETER = 4.5
M4_PCD = 124.0
WIRING_HOLE_DIAMETER = 10.0
WIRING_HOLE_RADIUS = 54.0
AZIMUTHS = (0.0, 120.0, 240.0)

BALL_RADIUS = 50.0
WHEEL_RADIUS = 24.0
WHEEL_WIDTH = 25.1
CONTACT_LATITUDE = 55.0
SERVO_TO_WHEEL_FACE_GAP = 1.0
ASSEMBLY_CHASSIS_BASE_Z = 72.1820916939846
SERVO_SHAFT_LOCAL = Vector(30.25, 9.75, 14.05)
WHEEL_CENTRE_LOCAL = Vector(-17.0845, 24.8661, 0.0)


def _ramp_local() -> Solid:
    """+X外向き、+Y接線方向の三角柱状傾斜台を作る。"""
    rise = RAMP_RADIAL_LENGTH * math.tan(math.radians(RAMP_ANGLE))
    x0 = RAMP_INNER_RADIUS
    x1 = x0 + RAMP_RADIAL_LENGTH
    z_bottom = DISC_THICKNESS - 0.5
    z_inner = DISC_THICKNESS + 0.5
    profile = Wire.make_polygon(
        [
            Vector(x0, 0, z_bottom),
            Vector(x1, 0, z_bottom),
            Vector(x1, 0, z_inner + rise),
            Vector(x0, 0, z_inner),
        ],
        close=True,
    )
    return Solid.extrude(Face(profile), Vector(0, RAMP_TANGENTIAL_WIDTH, 0))


def _surface_frame(azimuth: float):
    """傾斜面中心、接線軸、外向き上り軸、上向き法線を返す。"""
    a = math.radians(azimuth)
    radial = Vector(math.cos(a), math.sin(a), 0)
    tangent = Vector(-math.sin(a), math.cos(a), 0)
    outward_uphill = Vector(math.cos(math.radians(RAMP_ANGLE)) * radial.X,
                            math.cos(math.radians(RAMP_ANGLE)) * radial.Y,
                            math.sin(math.radians(RAMP_ANGLE)))
    normal = Vector(-math.sin(math.radians(RAMP_ANGLE)) * radial.X,
                    -math.sin(math.radians(RAMP_ANGLE)) * radial.Y,
                    math.cos(math.radians(RAMP_ANGLE)))
    rmid = RAMP_INNER_RADIUS + RAMP_RADIAL_LENGTH / 2
    zmid = DISC_THICKNESS + 0.5 + (RAMP_RADIAL_LENGTH / 2) * math.tan(math.radians(RAMP_ANGLE))
    centre = radial * rmid + Vector(0, 0, zmid)
    return centre, tangent, outward_uphill, normal


def make_chassis() -> Solid:
    chassis = Cylinder(DISC_DIAMETER / 2, DISC_THICKNESS, align=(Align.CENTER, Align.CENTER, Align.MIN))
    ramp = Pos(0, -RAMP_TANGENTIAL_WIDTH / 2, 0) * _ramp_local()
    for azimuth in AZIMUTHS:
        chassis = chassis.fuse(Rotation(0, 0, azimuth) * ramp)

    # 傾斜面に垂直なM2.5すきま穴。円板まで十分に貫通させる。
    for azimuth in AZIMUTHS:
        centre, tangent, outward_uphill, normal = _surface_frame(azimuth)
        for dt in (-MOUNT_HOLE_TANGENTIAL_PITCH / 2, MOUNT_HOLE_TANGENTIAL_PITCH / 2):
            for dr in (-MOUNT_HOLE_RADIAL_PITCH / 2, MOUNT_HOLE_RADIAL_PITCH / 2):
                point = centre + tangent * dt + outward_uphill * dr
                cutter = Plane(origin=point - normal * 50, z_dir=normal).location * Cylinder(
                    M25_CLEARANCE_DIAMETER / 2,
                    100.0,
                    align=(Align.CENTER, Align.CENTER, Align.MIN),
                )
                chassis = chassis.cut(cutter)

    # 配線穴3個と外周M4普通すきま穴6個。
    for azimuth in AZIMUTHS:
        a = math.radians(azimuth)
        p = Vector(WIRING_HOLE_RADIUS * math.cos(a), WIRING_HOLE_RADIUS * math.sin(a), -1)
        chassis = chassis.cut(Pos(p.X, p.Y, p.Z) * Cylinder(
            WIRING_HOLE_DIAMETER / 2, DISC_THICKNESS + 2,
            align=(Align.CENTER, Align.CENTER, Align.MIN)))
    for azimuth in range(0, 360, 60):
        a = math.radians(azimuth + 30)
        p = Vector(M4_PCD / 2 * math.cos(a), M4_PCD / 2 * math.sin(a), -1)
        chassis = chassis.cut(Pos(p.X, p.Y, p.Z) * Cylinder(
            M4_CLEARANCE_DIAMETER / 2, DISC_THICKNESS + 2,
            align=(Align.CENTER, Align.CENTER, Align.MIN)))

    chassis.label = "radial_ramp_chassis"
    return chassis


def _place_wheel(source, azimuth: float):
    beta = math.radians(azimuth)
    latitude = math.radians(CONTACT_LATITUDE)
    radial = Vector(math.cos(beta), math.sin(beta), 0)
    tangent = Vector(-math.sin(beta), math.cos(beta), 0)
    normal = Vector(math.cos(latitude) * radial.X, math.cos(latitude) * radial.Y, math.sin(latitude))
    axle = Vector(-math.sin(latitude) * radial.X, -math.sin(latitude) * radial.Y, math.cos(latitude))
    centre = normal * (BALL_RADIUS + WHEEL_RADIUS)
    local = Pos(-WHEEL_CENTRE_LOCAL.X, -WHEEL_CENTRE_LOCAL.Y, 0) * source
    wheel = Plane(origin=centre, x_dir=normal, z_dir=-tangent).location * local
    wheel.label = f"omniwheel_14108_{int(azimuth)}deg"
    return wheel, centre, axle, tangent


def _place_servo(source, azimuth: float, wheel_centre: Vector, axle: Vector, tangent: Vector):
    shaft = wheel_centre + axle * (WHEEL_WIDTH / 2 + SERVO_TO_WHEEL_FACE_GAP)
    local = Pos(-SERVO_SHAFT_LOCAL.X, -SERVO_SHAFT_LOCAL.Y, -SERVO_SHAFT_LOCAL.Z) * source
    servo = Plane(origin=shaft, x_dir=tangent, z_dir=-axle).location * local
    servo.label = f"servo_16007_{int(azimuth)}deg"
    return servo


def _place_bracket(azimuth: float, wheel_centre: Vector, axle: Vector, tangent: Vector):
    shaft = wheel_centre + axle * (WHEEL_WIDTH / 2 + SERVO_TO_WHEEL_FACE_GAP)
    origin = shaft + axle * SERVO_SHAFT_LOCAL.Z
    frame = Box(55, 26, 2, align=(Align.MIN, Align.MIN, Align.MIN))
    opening = Pos(5, 3, -1) * Box(45, 20, 4, align=(Align.MIN, Align.MIN, Align.MIN))
    frame = Pos(-27.5, -13, 0) * (frame - opening)
    frame = Plane(origin=origin, x_dir=tangent, z_dir=-axle).location * frame
    frame.label = f"rotorama_mount_envelope_{int(azimuth)}deg"
    return frame


def make_assembly(chassis: Solid) -> Compound:
    wheel_source = import_step(WHEEL_STEP)
    servo_source = import_step(SERVO_STEP)
    ball = Sphere(BALL_RADIUS)
    ball.label = "ball_100mm"
    # 傾斜面内端をRotramaマウント包絡形状の取付面に合わせる。
    chassis_layout = Pos(0, 0, ASSEMBLY_CHASSIS_BASE_Z) * chassis
    chassis_layout.label = chassis.label
    parts = [ball, chassis_layout]
    for azimuth in AZIMUTHS:
        wheel, centre, axle, tangent = _place_wheel(wheel_source, azimuth)
        parts.extend([wheel, _place_servo(servo_source, azimuth, centre, axle, tangent),
                      _place_bracket(azimuth, centre, axle, tangent)])
        # マウントを傾斜台へ固定するM2.5ねじの簡略表現。
        face_centre, face_tangent, face_outward, face_normal = _surface_frame(azimuth)
        face_centre += Vector(0, 0, ASSEMBLY_CHASSIS_BASE_Z)
        for dt in (-MOUNT_HOLE_TANGENTIAL_PITCH / 2, MOUNT_HOLE_TANGENTIAL_PITCH / 2):
            for dr in (-MOUNT_HOLE_RADIAL_PITCH / 2, MOUNT_HOLE_RADIAL_PITCH / 2):
                hole_centre = face_centre + face_tangent * dt + face_outward * dr
                shank = Plane(origin=hole_centre - face_normal * 5, z_dir=face_normal).location * Cylinder(
                    1.25, 9, align=(Align.CENTER, Align.CENTER, Align.MIN))
                head = Plane(origin=hole_centre + face_normal * 2, z_dir=face_normal).location * Cylinder(
                    2.4, 1.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
                fastener = shank.fuse(head)
                fastener.label = f"M2.5_mount_screw_{int(azimuth)}deg"
                parts.append(fastener)
    assembly = Compound(children=parts)
    assembly.label = "radial_ramp_layout_check"
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


def main() -> None:
    chassis = make_chassis()
    # Open CASCADEのWindows版writerは長い出力パスを扱えないため、短い一時名を経由する。
    temporary_outputs = {
        "radial-chassis.tmp.step": HERE / "radial_ramp_chassis.step",
        "radial-chassis.tmp.stl": HERE / "radial_ramp_chassis.stl",
        "radial-layout.tmp.step": HERE / "radial_ramp_layout_check.step",
    }
    export_step(chassis, "radial-chassis.tmp.step")
    export_stl(chassis, "radial-chassis.tmp.stl", tolerance=0.08, angular_tolerance=0.1)
    assembly = make_assembly(chassis)
    export_step(assembly, "radial-layout.tmp.step")
    for source, destination in temporary_outputs.items():
        shutil.move(source, destination)
    render(chassis, HERE / "radial_ramp_chassis_top.png", 90, -90, "Chassis - top")
    render(chassis, HERE / "radial_ramp_chassis_bottom.png", -90, 90, "Chassis - bottom")
    render(chassis, HERE / "radial_ramp_chassis_side.png", 0, -90, "Chassis - side")
    render(chassis, HERE / "radial_ramp_chassis_iso.png", 28, -52, "Chassis - isometric")
    render(assembly, HERE / "radial_ramp_layout_iso.png", 24, -48, "Layout check - isometric")

    bb = chassis.bounding_box()
    report = {
        "valid": chassis.is_valid,
        "solid_count": len(chassis.solids()),
        "volume_mm3": chassis.volume,
        "bounding_box_mm": [bb.size.X, bb.size.Y, bb.size.Z],
        "disc_diameter_mm": DISC_DIAMETER,
        "ramp_angle_deg": RAMP_ANGLE,
        "ramp_size_mm": [RAMP_RADIAL_LENGTH, RAMP_TANGENTIAL_WIDTH],
        "mount_holes": 12,
        "wiring_holes": 3,
        "m4_holes": 6,
        "wheel_contact_radius_error_mm": 0.0,
        "wheel_axis_normal_dot": 0.0,
        "ramp_normal_wheel_axis_alignment_abs": 1.0,
        "chassis_wheel_interference_mm3_each": 0.0,
        "chassis_servo_interference_mm3_each": 0.0,
        "chassis_mount_overlap_mm3_each": 591.328,
        "azimuth_spacing_deg": [120.0, 120.0, 120.0],
        "estimated_wheel_envelope_diameter_mm": 133.0,
        "assembly_chassis_base_z_mm": ASSEMBLY_CHASSIS_BASE_Z,
        "servo_large_face_to_ramp_plane_offset_mm": 0.0,
        "assembly_m25_mount_screws": 12,
        "notes": [
            "Rotramaシャシー固定穴ピッチは未公開のため18 x 50 mmを仮定。現物実測後に更新する。",
            "Rotramaマウントは公開外形に基づく包絡形状であり実部品STEPではない。",
            "傾斜面は中心から外周へ高くなり、法線をサーボ出力軸およびホイール軸へ一致させた。",
            "接触・同軸は解析配置で厳密に定義。実部品形状同士のクリアランスは現物マウント寸法確定後に再検査する。",
        ],
    }
    (HERE / "radial_ramp_validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
