"""camelCase/PascalCase → snake_case、kPascal → UPPER_SNAKE の一括リネーム。

自前の src/ 内識別子のみを対象とする。HAL/Zephyr 等の外部シンボルは
スキップ対象リストで除外する。
"""
import re
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"

# HAL/CMSIS/Zephyr/C++キーワード等、変換しない識別子のプレフィックス/完全一致
SKIP_PREFIX = (
    "HAL_", "__HAL_", "TIM_", "GPIO_", "GPIO_PIN", "I2C", "USART", "SPI",
    "RCC_", "EXTI", "IRQ", "NVIC", "LL_", "DWT", "USE_", "PIN_", "AF",
    "TIM_IT", "TIM_CHANNEL", "TIM_COUNTER", "TIM_ENCODER", "TIM_OCPOL",
    "UART_", "I2C_MEM", "assert_", "SNOR_", "FLASH_", "PWR_", "CRC_",
    "ETL_", "Z_", "K_", "DT_", "CONFIG_", "GPIO_DT", "PWM_", "SENSOR_",
    "LOG_", "BIT", "MIN", "MAX", "MAX_", "UINT", "INT", "TRUE", "FALSE",
    "K_PRIO", "SYS_", "DEVICE_", "BUILD_", "COND_CODE", "ARRAY_",
)
SKIP_EXACT = {
    "F", "U", "UL", "IRQn_Type", "TIM_HandleTypeDef", "GPIO_TypeDef",
    "I2C_HandleTypeDef", "UART_HandleTypeDef", "GPIO_InitTypeDef",
    "TIM_OC_InitTypeDef", "GPIO_PinState", "EXTI_HandleTypeDef",
    "Vec2",  # 別途マップで扱う
}
FORCE_MAP = {
    # 型エイリアス・クラス名の確定マップ
    "Vec2": "vec2", "Vec3": "vec3", "Vec4": "vec4", "Vec5": "vec5",
    "Vec10": "vec10", "Mat33": "mat33", "Mat55": "mat55",
    "Params": "params", "Ekf": "ekf", "Geom": "geom", "Lqi": "lqi",
    "TorqueLqi": "torque_lqi", "TorqueDrive": "torque_drive",
    "Rover": "rover", "Motor": "motor", "Driver": "driver",
    "Encoder": "encoder", "Command": "command", "Config": "config",
    "Input": "input", "Output": "output", "State": "state",
    "OutputState": "output_state", "Edge": "edge", "Physical": "physical",
    "Estimator": "estimator", "Controller": "controller",
    "YawGate": "yaw_gate", "GpioPin": "gpio_pin", "PwmOutput": "pwm_output",
    "EncoderCounter": "encoder_counter", "I2cBus": "i2c_bus",
    "Timebase": "timebase", "DebugUart": "debug_uart",
    "Lsm6dso": "lsm6dso", "WheelEncoder": "wheel_encoder",
    "Mdd3aMotor": "mdd3a_motor", "StatusLed": "status_led",
    "EstopInput": "estop_input", "StateEstimator": "state_estimator",
    "BalanceController": "balance_controller", "WheelServo": "wheel_servo",
    "CommandProfile": "command_profile",
    "WheelSpeedMeter": "wheel_speed_meter",
    "RobotApplication": "robot_application",
    "TaskScheduler": "task_scheduler",
}

ID_RE = re.compile(r"\b[A-Za-z_][A-Za-z0-9_]*\b")


def camel_to_snake(name: str) -> str:
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", name)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    return s.lower()


def is_constant_k(name: str) -> bool:
    return len(name) > 1 and name[0] == "k" and name[1].isupper()


def convert(name: str) -> str:
    if name in FORCE_MAP:
        return FORCE_MAP[name]
    if is_constant_k(name):
        return camel_to_snake(name[1:]).upper()
    return camel_to_snake(name)


def skip(name: str) -> bool:
    if name in SKIP_EXACT:
        return True
    if name.isupper():
        return True
    for p in SKIP_PREFIX:
        if name.startswith(p):
            return True
    return False


def main() -> None:
    mapping = {}
    files = [p for p in SRC.rglob("*") if p.suffix in (".hpp", ".cpp", ".h", ".c")]
    for path in files:
        text = path.read_text(encoding="utf-8")
        for m in ID_RE.finditer(text):
            tok = m.group(0)
            if skip(tok) or tok.islower():
                continue
            if tok in mapping:
                continue
            mapping[tok] = convert(tok)
    # 長い名前から置換（部分一致防止は \b で担保）
    items = sorted(mapping.items(), key=lambda kv: -len(kv[0]))
    pat = re.compile(r"\b(" + "|".join(re.escape(k) for k, _ in items) + r")\b")
    table = dict(items)
    for path in files:
        text = path.read_text(encoding="utf-8")
        out = pat.sub(lambda m: table[m.group(0)], text)
        if out != text:
            path.write_text(out, encoding="utf-8")
    for k, v in items:
        print(f"{k} -> {v}")
    print(f"total: {len(items)}")


if __name__ == "__main__":
    main()
