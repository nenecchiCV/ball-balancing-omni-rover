"""Full 3WD layout using the new underslung chassis and supplied components."""

import math
import runpy
from pathlib import Path

from build123d import Align, Box, Compound, Plane, Pos, Sphere, Vector
from cadgen.step_scene import import_step


BALL_RADIUS = 50.0
WHEEL_RADIUS = 24.0
WHEEL_WIDTH = 25.1
CONTACT_LATITUDE_DEG = 55.0
AZIMUTHS_DEG = (0.0, 120.0, 240.0)
SERVO_TO_WHEEL_FACE_GAP = 1.0

SERVO_SHAFT_TOP_LOCAL = Vector(30.25, 9.75, 14.05)
WHEEL_CENTRE_LOCAL = Vector(-17.0845, 24.8661, 0.0)

HERE = Path(__file__).resolve().parent
MECHANISM_ROOT = HERE.parents[1]
SERVO_STEP = MECHANISM_ROOT / "omnirover3wd_reference" / "servo_16007" / "mg996r_like_16007.step"
WHEEL_STEP = MECHANISM_ROOT / "omnirover3wd_reference" / "omniwheel_14108" / "48MM-OMNI-WHEEL-for-NXT-and-servo-motor 14108" / "48MM-OMNI-WHEEL-for-NXT-and-servo-motor.stp"


def _vectors(azimuth_deg: float):
    beta = math.radians(azimuth_deg)
    latitude = math.radians(CONTACT_LATITUDE_DEG)
    radial = Vector(math.cos(beta), math.sin(beta), 0.0)
    tangent = Vector(-math.sin(beta), math.cos(beta), 0.0)
    normal = Vector(
        math.cos(latitude) * math.cos(beta),
        math.cos(latitude) * math.sin(beta),
        math.sin(latitude),
    )
    axle = Vector(
        -math.sin(latitude) * math.cos(beta),
        -math.sin(latitude) * math.sin(beta),
        math.cos(latitude),
    )
    return radial, tangent, normal, axle


def _load_chassis():
    source = runpy.run_path(str(HERE / "nexus16007_chassis.step.py"))
    return source["gen_step"]()


def _place_wheel(source, azimuth_deg: float):
    _, tangent, normal, axle = _vectors(azimuth_deg)
    centre = normal * (BALL_RADIUS + WHEEL_RADIUS)
    local = Pos(-WHEEL_CENTRE_LOCAL.X, -WHEEL_CENTRE_LOCAL.Y, 0.0) * source
    wheel = Plane(origin=centre, x_dir=normal, z_dir=-tangent).location * local
    wheel.label = f"omniwheel_14108:{int(azimuth_deg)}deg"
    return wheel, centre, axle, tangent, normal


def _place_servo(source, azimuth_deg: float, wheel_centre, axle, tangent):
    shaft = wheel_centre + axle * (WHEEL_WIDTH / 2.0 + SERVO_TO_WHEEL_FACE_GAP)
    local = Pos(
        -SERVO_SHAFT_TOP_LOCAL.X,
        -SERVO_SHAFT_TOP_LOCAL.Y,
        -SERVO_SHAFT_TOP_LOCAL.Z,
    ) * source
    # Servo local X (its long edge) maps to the horizontal tangent.  Local Z
    # maps to -axle, keeping the output shaft coaxial with the wheel.
    servo = Plane(origin=shaft, x_dir=tangent, z_dir=-axle).location * local
    servo.label = f"servo_16007_proxy:{int(azimuth_deg)}deg"
    return servo


def _place_bracket(azimuth_deg: float, wheel_centre, axle, tangent):
    """Rotrama holder envelope aligned with the servo mounting-ear plane."""
    shaft = wheel_centre + axle * (WHEEL_WIDTH / 2.0 + SERVO_TO_WHEEL_FACE_GAP)
    mount_origin = shaft + axle * SERVO_SHAFT_TOP_LOCAL.Z
    outer = Box(55.0, 26.0, 2.0, align=(Align.MIN, Align.MIN, Align.MIN))
    outer = Pos(-27.5, -13.0, 0.0) * outer
    opening = Box(45.0, 20.0, 4.0, align=(Align.MIN, Align.MIN, Align.MIN))
    opening = Pos(-22.5, -10.0, -1.0) * opening
    frame = outer - opening
    frame = Plane(origin=mount_origin, x_dir=tangent, z_dir=-axle).location * frame
    frame.label = f"rotorama_servo_mount_envelope:{int(azimuth_deg)}deg"
    return frame


def gen_step():
    servo_source = import_step(SERVO_STEP)
    wheel_source = import_step(WHEEL_STEP)

    ball = Sphere(BALL_RADIUS)
    ball.label = "rigid_ball_100mm_285g_design_basis"
    chassis = _load_chassis()
    children = [ball, chassis]

    for azimuth in AZIMUTHS_DEG:
        wheel, centre, axle, tangent, _ = _place_wheel(wheel_source, azimuth)
        servo = _place_servo(servo_source, azimuth, centre, axle, tangent)
        bracket = _place_bracket(azimuth, centre, axle, tangent)
        children.extend([servo, bracket, wheel])

    assembly = Compound(children=children)
    assembly.label = "ball_balancing_omnirover_full_assembly"
    return assembly
