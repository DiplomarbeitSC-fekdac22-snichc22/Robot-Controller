from dataclasses import dataclass
from pathlib import Path
import math
import tomllib
from typing import Any


# GPIO (BCM numbering)
START_BUTTON_PIN = 17
STOP_BUTTON_PIN = 27
TEST_BUTTON_PIN = 22
LED_RELAY_PIN = 23

# I2C devices
LCD_ADDRESS = 0x27
LCD_COLUMNS = 20
LCD_ROWS = 4
PCA9685_ADDRESS = 0x40

# PS2 controller on /dev/spidev0.0 (CE0 is the ATT/select line)
PS2_SPI_BUS = 0
PS2_SPI_DEVICE = 0
PS2_SPI_SPEED_HZ = 250_000

# Timing
GAME_DURATION_SECONDS = 60.0
END_MESSAGE_SECONDS = 2.0
TEST_HOLD_SECONDS = 2.0
MOVE_INTERVAL_SECONDS = 0.030
DISPLAY_UPDATE_SECONDS = 0.500
CONTROLLER_TIMEOUT_SECONDS = 0.500
CONTROLLER_RETRY_SECONDS = 0.500
LOOP_DELAY_SECONDS = 0.015

# Keyboard control
KEYBOARD_STEP_DEGREES = 1

# Analog-stick dead zone
DEAD_ZONE_MIN = 110
DEAD_ZONE_MAX = 145

SERVO_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "servos.toml"


@dataclass(frozen=True)
class ServoConfig:
    joint_name: str
    channel: int
    initial_angle: int
    minimum_angle: int
    maximum_angle: int
    speed: int
    direction: int
    servo_model: str
    pulse_min_us: float
    neutral_pulse_us: float
    pulse_max_us: float


def _require_table(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(f"{SERVO_CONFIG_PATH}: missing [{key}] table")
    return value


def _require_int(table: dict[str, Any], key: str, context: str) -> int:
    value = table.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{context}.{key} must be an integer")
    result = int(value)
    if result != value:
        raise RuntimeError(f"{context}.{key} must be an integer")
    return result


def _require_float(table: dict[str, Any], key: str, context: str) -> float:
    value = table.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{context}.{key} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise RuntimeError(f"{context}.{key} must be finite")
    return result


def _count_to_microseconds(count: int, frequency_hz: int, resolution: int) -> float:
    return count * (1_000_000.0 / frequency_hz) / resolution


def _load_servo_settings() -> tuple[
    int,
    int,
    int,
    dict[str, ServoConfig],
    ServoConfig,
    int,
    int,
]:
    try:
        with SERVO_CONFIG_PATH.open("rb") as handle:
            data = tomllib.load(handle)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise RuntimeError(f"Could not load {SERVO_CONFIG_PATH}: {error}") from error

    defaults = _require_table(data, "defaults")
    joints = _require_table(data, "joints")

    frequency_hz = _require_int(defaults, "pwm_frequency_hz", "defaults")
    resolution = _require_int(defaults, "pca9685_resolution_counts", "defaults")
    pulse_count_min = _require_int(defaults, "pulse_count_min", "defaults")
    pulse_count_max = _require_int(defaults, "pulse_count_max", "defaults")

    if frequency_hz <= 0:
        raise RuntimeError("defaults.pwm_frequency_hz must be greater than zero")
    if resolution <= 0:
        raise RuntimeError("defaults.pca9685_resolution_counts must be greater than zero")
    if not 0 <= pulse_count_min < pulse_count_max < resolution:
        raise RuntimeError(
            "defaults pulse counts must satisfy "
            "0 <= pulse_count_min < pulse_count_max < resolution"
        )

    expected_min_us = _count_to_microseconds(
        pulse_count_min,
        frequency_hz,
        resolution,
    )
    expected_max_us = _count_to_microseconds(
        pulse_count_max,
        frequency_hz,
        resolution,
    )
    neutral_count = pulse_count_min + (pulse_count_max - pulse_count_min) // 2
    expected_neutral_us = _count_to_microseconds(
        neutral_count,
        frequency_hz,
        resolution,
    )

    servos: dict[str, ServoConfig] = {}
    gripper_servo: ServoConfig | None = None
    gripper_open_angle: int | None = None
    gripper_closed_angle: int | None = None
    used_channels: set[int] = set()

    for joint_name, raw_joint in joints.items():
        if not isinstance(raw_joint, dict):
            raise RuntimeError(f"joints.{joint_name} must be a table")
        context = f"joints.{joint_name}"

        controller_name = raw_joint.get("controller_name")
        if not isinstance(controller_name, str) or not controller_name:
            raise RuntimeError(f"{context}.controller_name must be a non-empty string")

        channel = _require_int(raw_joint, "channel", context)
        if not 0 <= channel <= 15:
            raise RuntimeError(f"{context}.channel must be between 0 and 15")
        if channel in used_channels:
            raise RuntimeError(f"PCA9685 channel {channel} is assigned more than once")
        used_channels.add(channel)

        direction = _require_int(raw_joint, "direction", context)
        if direction not in (-1, 1):
            raise RuntimeError(f"{context}.direction must be -1 or 1")
        if direction != 1:
            raise RuntimeError(
                f"{context}.direction={direction} is not supported by the legacy angle mapper"
            )

        initial_angle = _require_int(raw_joint, "initial_angle_deg", context)
        minimum_angle = _require_int(raw_joint, "angle_min_deg", context)
        maximum_angle = _require_int(raw_joint, "angle_max_deg", context)
        speed = _require_int(raw_joint, "speed_deg_per_update", context)
        if minimum_angle > maximum_angle:
            raise RuntimeError(f"{context} angle minimum exceeds angle maximum")
        if not minimum_angle <= initial_angle <= maximum_angle:
            raise RuntimeError(f"{context} initial angle is outside its limits")
        if speed <= 0:
            raise RuntimeError(f"{context}.speed_deg_per_update must be positive")

        servo_model = raw_joint.get("servo_model")
        if not isinstance(servo_model, str) or not servo_model:
            raise RuntimeError(f"{context}.servo_model must be a non-empty string")

        pulse_min_us = _require_float(raw_joint, "pulse_min_us", context)
        neutral_pulse_us = _require_float(raw_joint, "neutral_pulse_us", context)
        pulse_max_us = _require_float(raw_joint, "pulse_max_us", context)
        for field_name, actual, expected in (
            ("pulse_min_us", pulse_min_us, expected_min_us),
            ("neutral_pulse_us", neutral_pulse_us, expected_neutral_us),
            ("pulse_max_us", pulse_max_us, expected_max_us),
        ):
            if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6):
                raise RuntimeError(
                    f"{context}.{field_name}={actual:g} does not match the legacy "
                    f"PCA9685 mapping value {expected:g}"
                )

        servo = ServoConfig(
            joint_name=joint_name,
            channel=channel,
            initial_angle=initial_angle,
            minimum_angle=minimum_angle,
            maximum_angle=maximum_angle,
            speed=speed,
            direction=direction,
            servo_model=servo_model,
            pulse_min_us=pulse_min_us,
            neutral_pulse_us=neutral_pulse_us,
            pulse_max_us=pulse_max_us,
        )

        if controller_name == "gripper":
            gripper_servo = servo
            gripper_open_angle = _require_int(raw_joint, "open_angle_deg", context)
            gripper_closed_angle = _require_int(raw_joint, "closed_angle_deg", context)
        else:
            if controller_name in servos:
                raise RuntimeError(
                    f"controller_name {controller_name!r} is assigned more than once"
                )
            servos[controller_name] = servo

    if gripper_servo is None:
        raise RuntimeError("No joint with controller_name='gripper' is configured")
    if gripper_open_angle is None or gripper_closed_angle is None:
        raise RuntimeError("Gripper open and closed angles are required")

    required_names = {"base", "shoulder", "elbow", "wrist"}
    missing_names = required_names - servos.keys()
    if missing_names:
        raise RuntimeError(
            "Missing controller servo(s): " + ", ".join(sorted(missing_names))
        )

    return (
        frequency_hz,
        pulse_count_min,
        pulse_count_max,
        servos,
        gripper_servo,
        gripper_open_angle,
        gripper_closed_angle,
    )


(
    PCA9685_FREQUENCY_HZ,
    SERVO_PULSE_MIN,
    SERVO_PULSE_MAX,
    SERVOS,
    GRIPPER_SERVO,
    GRIPPER_R1_ANGLE,
    GRIPPER_L1_ANGLE,
) = _load_servo_settings()

GRIPPER_CHANNEL = GRIPPER_SERVO.channel
