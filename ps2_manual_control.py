from __future__ import annotations

import argparse
import json
import math
import queue
import signal
import sys
import threading
import time
import tomllib
from types import SimpleNamespace
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ps2_controller import ControllerState, Ps2Controller
from game_display import GameDisplay


PSB_TOGGLE = 1 << 4
PSB_DISABLE = 1 << 6
PSB_L1 = 1 << 10
PSB_R1 = 1 << 11
PSB_MODE = 1 << 0
PSB_CONFIRM = 1 << 3

HOME_RETURN_MIN_SPEED_US_PER_SECOND = 300.0
GRIPPER_RELEASE_SECONDS = 0.5

RETURN_AXES = (
    "base",
    "shoulder",
    "elbow",
    "wrist",
)

DIRECT_HOME_MAX_DISTANCE_US = 300.0
SAFE_ROUTE_MATCH_MAX_DISTANCE_US = 375.0

GLASS_DOOR_BASE_MAX_US = 1711.0
GLASS_GUARD_BASE_START_US = 1500.0
GLASS_GUARD_SHOULDER_START_US = 1950.0
GLASS_GUARD_ELBOW_MIN_US = 1900.0

MEASURED_DIRECT_HOME_REGIONS = (
    (
        "safe_back_region",
        {
            "base": (1350.0, 1650.0),
            "shoulder": (2050.0, 2350.0),
            "elbow": (1950.0, 2250.0),
            "wrist": (1000.0, 1350.0),
        },
    ),
)

SAFE_RETURN_ROUTES = (
    (
        "upper_shelf_original",
        (975, 1465, 2312, 1407),
        (
            (975, 1465, 2362, 1488),
            (975, 1465, 2362, 1627),
            (975, 1490, 2362, 1478),
            (975, 1527, 2362, 1367),
            (975, 1937, 2362, 1367),
        ),
    ),
    (
        "edge_case_1",
        (981, 1467, 2240, 1551),
        (
            (981, 1467, 2389, 1624),
            (981, 1467, 2479, 1731),
            (981, 1871, 2479, 1665),
        ),
    ),
    (
        "edge_case_2",
        (970, 1248, 2546, 2076),
        (
            (970, 1321, 2546, 1758),
            (970, 1448, 2546, 1683),
            (970, 1448, 2546, 1301),
            (970, 1820, 2546, 1301),
        ),
    ),
    (
        "edge_case_3",
        (1191, 1367, 2143, 1573),
        (
            (1191, 1367, 2286, 1573),
            (1191, 1367, 2286, 1434),
            (1191, 1463, 2286, 1250),
            (1191, 1543, 2286, 1118),
            (1191, 1833, 2286, 1118),
        ),
    ),
    (
        "edge_case_4",
        (1121, 1309, 2107, 1789),
        (
            (1121, 1309, 2461, 1555),
            (1121, 1357, 2458, 1392),
            (1140, 1470, 2458, 1284),
            (1140, 1799, 2458, 1284),
        ),
    ),
    (
        "edge_case_5",
        (1199, 1338, 1814, 1129),
        (
            (1199, 1338, 1814, 909),
            (1199, 1338, 1729, 732),
            (1199, 1667, 1729, 732),
        ),
    ),
    (
        "edge_case_6",
        (771, 1303, 1808, 1172),
        (
            (771, 1303, 1911, 1172),
            (771, 1463, 1911, 1172),
            (771, 1463, 1920, 941),
            (762, 1607, 1920, 941),
        ),
    ),
    (
        "bottom_shelf",
        (1247, 1072, 1405, 1043),
        (
            (1247, 1072, 1384, 1156),
            (1247, 1448, 1384, 1156),
        ),
    ),
)

KINEMATICS_CONFIG_DIR = (
    Path.home() / "dev" / "Kinematics" / "configs"
)

KINEMATICS_JOINTS = {
    "base": "J1_base",
    "shoulder": "J2_shoulder",
    "elbow": "J3_elbow",
    "wrist": "J4_wrist",
}


@dataclass(frozen=True)
class AxisServo:
    name: str
    channel: int
    axis: str
    invert: int
    minimum_pulse_us: float
    start_pulse_us: float
    maximum_pulse_us: float
    speed_us_per_second: float


@dataclass(frozen=True)
class Gripper:
    channel: int
    open_pulse_us: float
    closed_pulse_us: float
    start_pulse_us: float
    speed_us_per_second: float = 350.0


@dataclass(frozen=True)
class Settings:
    frequency_hz: int
    i2c_address: int
    loop_hz: float
    status_hz: float
    dead_zone: int
    disconnect_timeout_seconds: float
    reconnect_interval_seconds: float
    axis_servos: tuple[AxisServo, ...]
    gripper: Gripper
    permission_seconds: float = 90.0
    end_message_seconds: float = 10.0
    motion_ramp_seconds: float = 0.25
    setup_button_gpio: int | None = None
    emergency_stop_gpio: int | None = None
    emergency_stop_healthy_pressed: bool | None = None
    control_api_enabled: bool = True
    control_api_port: int = 8765


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fail-safe PS2 manual control for the robot arm."
    )

    parser.add_argument(
        "--config",
        type=Path,
        help=(
            "Controller TOML path. Defaults to "
            "config/ps2_controller.toml beside this script."
        ),
    )

    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="Read and display PS2 input without touching any servo.",
    )
    parser.add_argument("--simulate", action="store_true",
                        help="Test buttons, modes and movement limits without opening any hardware.")

    return parser.parse_args()


def require_table(
    data: dict[str, Any],
    key: str,
) -> dict[str, Any]:
    value = data.get(key)

    if not isinstance(value, dict):
        raise RuntimeError(
            f"Missing TOML table [{key}]"
        )

    return value


def require_number(
    table: dict[str, Any],
    key: str,
    context: str,
) -> float:
    value = table.get(key)

    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
    ):
        raise RuntimeError(
            f"{context}.{key} must be numeric"
        )

    result = float(value)

    if not math.isfinite(result):
        raise RuntimeError(
            f"{context}.{key} must be finite"
        )

    return result


def require_integer(
    table: dict[str, Any],
    key: str,
    context: str,
) -> int:
    result = require_number(
        table,
        key,
        context,
    )

    integer = int(result)

    if integer != result:
        raise RuntimeError(
            f"{context}.{key} must be an integer"
        )

    return integer


def load_settings(
    path: Path,
) -> Settings:
    try:
        with path.open("rb") as stream:
            data = tomllib.load(stream)
    except (
        OSError,
        tomllib.TOMLDecodeError,
    ) as error:
        raise RuntimeError(
            f"Could not load {path}: {error}"
        ) from error

    controller = require_table(
        data,
        "controller",
    )
    servo_board = require_table(
        data,
        "servo_board",
    )
    servo_tables = require_table(
        data,
        "servos",
    )
    gripper_table = require_table(
        data,
        "gripper",
    )
    operation = data.get("operation", {})
    if not isinstance(operation, dict):
        raise RuntimeError("operation must be a TOML table")
    operation = {"permission_seconds": 90.0, "end_message_seconds": 10.0,
                 "motion_ramp_seconds": 0.25, "control_api_enabled": True,
                 "control_api_port": 8765, **operation}
    for key in ("permission_seconds", "end_message_seconds", "motion_ramp_seconds"):
        if require_number(operation, key, "operation") <= 0:
            raise RuntimeError(f"operation.{key} must be positive")
    for key in ("setup_button_gpio", "emergency_stop_gpio"):
        if key in operation:
            operation[key] = require_integer(operation, key, "operation")
            if not 0 <= operation[key] <= 27:
                raise RuntimeError(f"operation.{key} must be a BCM GPIO from 0 to 27")
    emergency_gpio = operation.get("emergency_stop_gpio")
    healthy_pressed = operation.get("emergency_stop_healthy_pressed")
    if (emergency_gpio is None) != (healthy_pressed is None) or (
        healthy_pressed is not None and healthy_pressed is not True
    ):
        raise RuntimeError("Configure both the E-stop feedback GPIO and verified healthy_pressed polarity")
    pins = [pin for pin in (operation.get("setup_button_gpio"), emergency_gpio) if pin is not None]
    if len(set(pins)) != len(pins) or any(pin in (2, 3, 5, 6, 8, 12, 13, 18, 25) for pin in pins):
        raise RuntimeError("Optional button GPIO conflicts with an existing controller connection")
    if type(operation["control_api_enabled"]) is not bool:
        raise RuntimeError("operation.control_api_enabled must be boolean")
    if not 1 <= require_integer(operation, "control_api_port", "operation") <= 65535:
        raise RuntimeError("operation.control_api_port must be from 1 to 65535")
    gripper_speed = gripper_table.get("speed_us_per_second", 350.0)
    if require_number({"speed": gripper_speed}, "speed", "gripper") <= 0:
        raise RuntimeError("gripper.speed_us_per_second must be positive")

    frequency_hz = require_integer(
        servo_board,
        "frequency_hz",
        "servo_board",
    )
    i2c_address = require_integer(
        servo_board,
        "i2c_address",
        "servo_board",
    )
    resolution_counts = require_integer(
        servo_board,
        "pca9685_resolution_counts",
        "servo_board",
    )
    pulse_count_min = require_number(
        servo_board,
        "pulse_count_min",
        "servo_board",
    )
    pulse_count_max = require_number(
        servo_board,
        "pulse_count_max",
        "servo_board",
    )
    mapping_angle_min = require_number(
        servo_board,
        "mapping_angle_min_deg",
        "servo_board",
    )
    mapping_angle_max = require_number(
        servo_board,
        "mapping_angle_max_deg",
        "servo_board",
    )
    loop_hz = require_number(
        controller,
        "loop_hz",
        "controller",
    )
    status_hz = require_number(
        controller,
        "status_hz",
        "controller",
    )
    dead_zone = require_integer(
        controller,
        "dead_zone",
        "controller",
    )
    disconnect_timeout_seconds = require_number(
        controller,
        "disconnect_timeout_seconds",
        "controller",
    )
    reconnect_interval_seconds = require_number(
        controller,
        "reconnect_interval_seconds",
        "controller",
    )

    if frequency_hz <= 0:
        raise RuntimeError(
            "servo_board.frequency_hz must be positive"
        )

    if not 0 <= i2c_address <= 0x7F:
        raise RuntimeError(
            "servo_board.i2c_address must be 0..127"
        )

    if resolution_counts <= 0:
        raise RuntimeError(
            "servo_board.pca9685_resolution_counts "
            "must be positive"
        )

    if pulse_count_min >= pulse_count_max:
        raise RuntimeError(
            "servo_board pulse_count_min must be "
            "below pulse_count_max"
        )

    if mapping_angle_min >= mapping_angle_max:
        raise RuntimeError(
            "servo_board.mapping_angle_min_deg must be "
            "below mapping_angle_max_deg"
        )

    if loop_hz <= 0 or status_hz <= 0:
        raise RuntimeError(
            "Controller frequencies must be positive"
        )

    if not 0 <= dead_zone < 127:
        raise RuntimeError(
            "controller.dead_zone must be 0..126"
        )

    if (
        disconnect_timeout_seconds <= 0
        or reconnect_interval_seconds <= 0
    ):
        raise RuntimeError(
            "Controller timeout values must be positive"
        )

    def initial_angle_to_pulse(
        table: dict[str, Any],
        context: str,
    ) -> float:
        initial_angle = require_number(
            table,
            "initial_angle_deg",
            context,
        )

        angle_fraction = (
            (initial_angle - mapping_angle_min)
            / (mapping_angle_max - mapping_angle_min)
        )

        pulse_count = (
            pulse_count_min
            + angle_fraction
            * (pulse_count_max - pulse_count_min)
        )

        period_us = (
            1_000_000.0 / frequency_hz
        )

        return (
            pulse_count
            / resolution_counts
            * period_us
        )

    axis_servos: list[AxisServo] = []
    used_channels: set[int] = set()

    valid_axes = {
        "left_x",
        "left_y",
        "right_x",
        "right_y",
    }

    for name in (
        "base",
        "shoulder",
        "elbow",
        "wrist",
    ):
        raw = servo_tables.get(name)
        context = f"servos.{name}"

        if not isinstance(raw, dict):
            raise RuntimeError(
                f"Missing TOML table [{context}]"
            )

        channel = require_integer(
            raw,
            "channel",
            context,
        )
        invert = require_integer(
            raw,
            "invert",
            context,
        )
        axis = raw.get("axis")
        minimum = require_number(
            raw,
            "minimum_pulse_us",
            context,
        )
        start = initial_angle_to_pulse(
            raw,
            context,
        )
        maximum = require_number(
            raw,
            "maximum_pulse_us",
            context,
        )
        speed = require_number(
            raw,
            "speed_us_per_second",
            context,
        )

        if not 0 <= channel <= 15:
            raise RuntimeError(
                f"{context}.channel must be 0..15"
            )

        if channel in used_channels:
            raise RuntimeError(
                f"PCA9685 channel {channel} is duplicated"
            )

        if axis not in valid_axes:
            raise RuntimeError(
                f"{context}.axis must be one of "
                f"{sorted(valid_axes)}"
            )

        if invert not in (-1, 1):
            raise RuntimeError(
                f"{context}.invert must be -1 or 1"
            )

        if not 500 <= minimum < maximum <= 3000:
            raise RuntimeError(
                f"{context} pulse range must satisfy "
                "500 <= minimum < maximum <= 3000"
            )

        if not minimum <= start <= maximum:
            raise RuntimeError(
                f"{context}.initial_angle_deg converts to "
                f"{start:.3f} us, which is outside its "
                "configured pulse limits"
            )

        if speed <= 0:
            raise RuntimeError(
                f"{context}.speed_us_per_second "
                "must be positive"
            )

        used_channels.add(channel)

        axis_servos.append(
            AxisServo(
                name=name,
                channel=channel,
                axis=axis,
                invert=invert,
                minimum_pulse_us=minimum,
                start_pulse_us=start,
                maximum_pulse_us=maximum,
                speed_us_per_second=speed,
            )
        )

    gripper_channel = require_integer(
        gripper_table,
        "channel",
        "gripper",
    )
    gripper_open = require_number(
        gripper_table,
        "open_pulse_us",
        "gripper",
    )
    gripper_closed = require_number(
        gripper_table,
        "closed_pulse_us",
        "gripper",
    )
    gripper_start = initial_angle_to_pulse(
        gripper_table,
        "gripper",
    )

    if not 0 <= gripper_channel <= 15:
        raise RuntimeError(
            "gripper.channel must be 0..15"
        )

    if gripper_channel in used_channels:
        raise RuntimeError(
            f"PCA9685 channel {gripper_channel} "
            "is duplicated"
        )

    for field_name, value in (
        ("open_pulse_us", gripper_open),
        ("closed_pulse_us", gripper_closed),
        ("start_pulse_us", gripper_start),
    ):
        if not 500 <= value <= 3000:
            raise RuntimeError(
                f"gripper.{field_name} must be 500..3000"
            )

    if gripper_start not in (
        gripper_open,
        gripper_closed,
    ):
        raise RuntimeError(
            "gripper.initial_angle_deg must convert "
            "exactly to the configured open or closed pulse"
        )

    return Settings(
        frequency_hz=frequency_hz,
        i2c_address=i2c_address,
        loop_hz=loop_hz,
        status_hz=status_hz,
        dead_zone=dead_zone,
        disconnect_timeout_seconds=(
            disconnect_timeout_seconds
        ),
        reconnect_interval_seconds=(
            reconnect_interval_seconds
        ),
        axis_servos=tuple(axis_servos),
        gripper=Gripper(
            channel=gripper_channel,
            open_pulse_us=gripper_open,
            closed_pulse_us=gripper_closed,
            start_pulse_us=gripper_start,
            speed_us_per_second=float(gripper_speed),
        ),
        permission_seconds=float(operation["permission_seconds"]),
        end_message_seconds=float(operation["end_message_seconds"]),
        motion_ramp_seconds=float(operation["motion_ramp_seconds"]),
        setup_button_gpio=operation.get("setup_button_gpio"),
        emergency_stop_gpio=emergency_gpio,
        emergency_stop_healthy_pressed=healthy_pressed,
        control_api_enabled=operation["control_api_enabled"],
        control_api_port=int(operation["control_api_port"]),
    )


@dataclass(frozen=True)
class KinematicsModel:
    l1_mm: float
    l2_mm: float
    lg_mm: float
    h0_mm: float
    base_x_mm: float
    base_y_mm: float
    base_z_mm: float
    y_positive_direction: str
    elbow_relative_sign: float
    theta_home_deg: dict[str, float]
    direction: dict[str, float]
    us_per_degree: dict[str, float]

    def gripper_xyz(
        self,
        pulses: dict[str, float],
        settings: Settings,
    ) -> tuple[float, float, float]:
        servos = {
            servo.name: servo
            for servo in settings.axis_servos
        }

        theta: dict[str, float] = {}

        for name in KINEMATICS_JOINTS:
            pulse_delta = (
                pulses[name]
                - servos[name].start_pulse_us
            )

            theta[name] = (
                self.theta_home_deg[name]
                + self.direction[name]
                * pulse_delta
                / self.us_per_degree[name]
            )

        theta1 = math.radians(
            theta["base"]
        )
        theta2 = math.radians(
            theta["shoulder"]
        )
        theta3 = math.radians(
            theta["elbow"]
        )
        theta4 = math.radians(
            theta["wrist"]
        )

        elbow_relative_angle = (
            self.elbow_relative_sign
            * (math.pi - theta3)
        )

        forearm_angle = (
            theta2 + elbow_relative_angle
        )
        gripper_angle = (
            forearm_angle + theta4
        )

        radial_mm = (
            self.l1_mm * math.cos(theta2)
            + self.l2_mm * math.cos(forearm_angle)
            + self.lg_mm * math.cos(gripper_angle)
        )

        gripper_y_math_mm = (
            self.l1_mm * math.sin(theta2)
            + self.l2_mm * math.sin(forearm_angle)
            + self.lg_mm * math.sin(gripper_angle)
        )

        if self.y_positive_direction == "down":
            shoulder_y_mm = (
                self.base_y_mm
                + self.h0_mm
            )

            y_mm = (
                shoulder_y_mm
                - gripper_y_math_mm
            )

        elif self.y_positive_direction == "up":
            y_mm = (
                self.h0_mm
                + gripper_y_math_mm
            )

        else:
            raise RuntimeError(
                "Unsupported y_positive_direction: "
                f"{self.y_positive_direction!r}"
            )

        x_mm = (
            self.base_x_mm
            + radial_mm * math.cos(theta1)
        )

        z_mm = (
            self.base_z_mm
            + radial_mm * math.sin(theta1)
        )

        return x_mm, y_mm, z_mm


def load_kinematics_model(
    config_dir: Path = KINEMATICS_CONFIG_DIR,
) -> KinematicsModel | None:
    try:
        with (
            config_dir / "robot_geometry.toml"
        ).open("rb") as stream:
            geometry = tomllib.load(stream)

        with (
            config_dir / "servo_calibration.toml"
        ).open("rb") as stream:
            calibration = tomllib.load(stream)

        with (
            config_dir / "kinematics_settings.toml"
        ).open("rb") as stream:
            kinematics = tomllib.load(stream)

        links = geometry["link_lengths_mm"]
        model = kinematics["model"]
        coordinates = kinematics[
            "input_coordinates"
        ]

        base = coordinates[
            "base_rotation_axis_at_mounting_plate_mm"
        ]

        theta_home_deg: dict[str, float] = {}
        direction: dict[str, float] = {}
        us_per_degree: dict[str, float] = {}

        for (
            controller_name,
            joint_name,
        ) in KINEMATICS_JOINTS.items():
            joint = calibration[
                "joints"
            ][joint_name]

            theta_home_deg[
                controller_name
            ] = float(
                joint["theta_home_deg"]
            )

            direction[
                controller_name
            ] = float(
                joint["direction"]
            )

            us_per_degree[
                controller_name
            ] = float(
                joint["us_per_degree"]
            )

        return KinematicsModel(
            l1_mm=float(
                links["L1_shoulder_to_elbow"]
            ),
            l2_mm=float(
                links["L2_elbow_to_wrist"]
            ),
            lg_mm=(
                float(
                    links[
                        model["selected_Lg_key"]
                    ]
                )
                if model["use_gripper_offset"]
                else 0.0
            ),
            h0_mm=(
                float(
                    links[
                        model["selected_h0_key"]
                    ]
                )
                if model[
                    "use_h0_from_robot_geometry"
                ]
                else 0.0
            ),
            base_x_mm=float(base[0]),
            base_y_mm=float(base[1]),
            base_z_mm=float(base[2]),
            y_positive_direction=str(
                coordinates[
                    "y_positive_direction"
                ]
            ),
            elbow_relative_sign=float(
                kinematics.get(
                    "fk",
                    {},
                ).get(
                    "elbow_relative_sign",
                    -1.0,
                )
            ),
            theta_home_deg=theta_home_deg,
            direction=direction,
            us_per_degree=us_per_degree,
        )

    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        tomllib.TOMLDecodeError,
    ) as error:
        print(
            "WARNING: gripper XYZ display "
            f"is unavailable: {error}"
        )

        return None


class PulseServoBoard:
    def __init__(
        self,
        settings: Settings,
    ) -> None:
        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685
        except ImportError as error:
            raise RuntimeError(
                "PCA9685 Python dependencies "
                "are not installed"
            ) from error

        self._frequency_hz = (
            settings.frequency_hz
        )

        self._channels = {
            servo.channel
            for servo in settings.axis_servos
        } | {
            settings.gripper.channel
        }

        self._i2c = busio.I2C(
            board.SCL,
            board.SDA,
        )

        self._pca = None
        try:
            self._pca = PCA9685(self._i2c, address=settings.i2c_address)
            self._pca.frequency = settings.frequency_hz
            self.disable_outputs()
        except Exception:
            try:
                if self._pca is not None:
                    self._pca.deinit()
            finally:
                self._i2c.deinit()
            raise

    def pulse_to_duty_cycle(
        self,
        pulse_us: float,
    ) -> int:
        period_us = (
            1_000_000.0
            / self._frequency_hz
        )

        counts = round(
            pulse_us
            / period_us
            * 4096.0
        )

        if not 0 <= counts <= 4095:
            raise ValueError(
                f"Pulse {pulse_us}us is outside "
                "the PCA9685 period"
            )

        return counts << 4

    def set_pulse(
        self,
        channel: int,
        pulse_us: float,
    ) -> None:
        self._pca.channels[
            channel
        ].duty_cycle = (
            self.pulse_to_duty_cycle(
                pulse_us
            )
        )

    def disable_outputs(self) -> None:
        failure = None
        for channel in self._channels:
            try:
                self._pca.channels[channel].duty_cycle = 0
            except Exception as error:
                failure = failure or error
        if failure is not None:
            raise failure

    def close(self) -> None:
        try:
            self.disable_outputs()
        finally:
            try:
                self._pca.deinit()
            finally:
                self._i2c.deinit()


def axis_amount(
    value: int,
    dead_zone: int,
) -> float:
    offset = max(
        -128,
        min(
            127,
            value - 128,
        ),
    )

    magnitude = abs(offset)

    if magnitude <= dead_zone:
        return 0.0

    scaled = (
        (magnitude - dead_zone)
        / (127.0 - dead_zone)
    )

    return math.copysign(
        min(1.0, scaled),
        offset,
    )


def print_controller_state(
    state: ControllerState,
    prefix: str,
) -> None:
    print(
        "\r"
        f"{prefix:<12} "
        f"LX={state.left_x:3d} "
        f"LY={state.left_y:3d} "
        f"RX={state.right_x:3d} "
        f"RY={state.right_y:3d} "
        f"buttons=0x{state.buttons:04X}",
        end="",
        flush=True,
    )


def run_diagnostics(
    settings: Settings,
) -> None:
    controller = Ps2Controller()
    next_retry = 0.0
    connected = False

    update_interval = (
        1.0 / settings.status_hz
    )

    next_update = 0.0

    print(
        "PS2 diagnostic mode: "
        "servos are not opened or controlled."
    )
    print(
        "Move the sticks and press buttons. "
        "Press Ctrl+C to stop."
    )

    try:
        while True:
            now = time.monotonic()

            if not connected:
                if now < next_retry:
                    time.sleep(0.01)
                    continue

                next_retry = (
                    now
                    + settings
                    .reconnect_interval_seconds
                )

                connected = (
                    controller.begin()
                )

                if not connected:
                    print(
                        "\rWaiting for PS2 controller...",
                        end="",
                        flush=True,
                    )
                    continue

                print(
                    "\nPS2 controller connected."
                )

            state = controller.read()

            if state is None:
                connected = False
                continue

            if now >= next_update:
                next_update = (
                    now + update_interval
                )

                print_controller_state(
                    state,
                    "DIAGNOSTIC",
                )

            time.sleep(
                1.0 / settings.loop_hz
            )

    except KeyboardInterrupt:
        print(
            "\nDiagnostic stopped."
        )

    finally:
        controller.close()


class ManualController:
    def __init__(
        self,
        settings: Settings,
        *, controller=None, board=None, game_display=None, simulated=False, clock=None,
    ) -> None:
        self.settings = settings
        self.clock = clock if clock is not None else time.monotonic
        self.controller = controller if controller is not None else Ps2Controller()

        self.home_required_after_stop = True

        self.returning_home = False
        self.return_waypoints: list[
            dict[str, float]
        ] = []
        self.return_waypoint_index = 0
        self.home_release_until = 0.0
        self.return_mode: str | None = None
        self.escape_only = False

        try:
            self.board = board if board is not None else PulseServoBoard(settings)
        except Exception:
            self.controller.close()
            raise

        self.pulses = {
            servo.name: servo.start_pulse_us
            for servo in settings.axis_servos
        }

        self.pulses["gripper"] = (
            settings.gripper.start_pulse_us
        )

        self.kinematics = None if simulated else load_kinematics_model()

        try:
            self.game_display = game_display if game_display is not None else GameDisplay(
                duration_seconds=settings.permission_seconds,
                notice_seconds=settings.end_message_seconds,
                setup_button_gpio=settings.setup_button_gpio,
                emergency_stop_gpio=settings.emergency_stop_gpio,
                emergency_stop_healthy_pressed=settings.emergency_stop_healthy_pressed,
            )
        except Exception:
            self.board.close()
            self.controller.close()
            raise

        self.armed = False
        self.operation_mode = "manual"
        self.game_display.operation_mode = "manual"
        self.position_confirmed = False
        self.simulated = simulated
        self._outputs_disabled = True
        self._mode_requests = queue.Queue(maxsize=1)
        self.api_server = None
        self.velocities = {servo.name: 0.0 for servo in settings.axis_servos}
        self.gripper_target = self.pulses["gripper"]
        self.last_stop_reason = "Confirm the actual arm pose with START, then grant permission"
        self.connected = False
        self.previous_buttons = 0
        self.last_good_read = 0.0
        self.next_retry = 0.0
        self.next_status = 0.0
        self.last_loop = self.clock()
        self.running = True
        self._closed = False

    def neutral(self, state: ControllerState) -> bool:
        return all(axis_amount(getattr(state, servo.axis), self.settings.dead_zone) == 0
                   for servo in self.settings.axis_servos) and not state.buttons & (PSB_L1 | PSB_R1 | PSB_DISABLE)

    def get_status(self) -> dict:
        now = self.clock()
        permission = self.game_display.permission
        granted = self.game_display.movement_allowed and (
            permission.state == "setup" or now < permission.ends_at
        )
        return {"mode": self.operation_mode, "permission": permission.state,
                "permission_remaining_seconds": max(0.0, permission.ends_at - now) if permission.state == "timed" else None,
                "armed": self.armed, "connected": self.connected,
                "movement_allowed": self.running and self.armed and granted and self.connected and self.operation_mode == "manual",
                "position_confirmed": self.position_confirmed,
                "emergency_stop_available": self.game_display.emergency_stop_available,
                "emergency_stop_active": self.game_display.emergency_stop_active,
                "automatic_ready": False, "simulated": self.simulated,
                "reason": self.last_stop_reason}

    def set_mode(self, mode: str, now=None) -> None:
        if mode not in ("manual", "automatic"):
            raise ValueError("Mode must be manual or automatic")
        if mode != self.operation_mode:
            self.disable("Operating mode changed; a fresh permission is required", now=now)
            self.operation_mode = mode
            self.game_display.operation_mode = mode
            self.game_display.refresh(self.clock() if now is None else now, False)
            print(f"Operating mode: {mode}; automatic pickup is not connected")

    def request_mode(self, mode: str) -> dict:
        if mode not in ("manual", "automatic"):
            raise RuntimeError("Mode must be manual or automatic")
        if not self.running:
            raise RuntimeError("Controller is stopped")
        completed = threading.Event()
        try:
            self._mode_requests.put_nowait((mode, completed, self.clock() + 0.3))
        except queue.Full as error:
            raise RuntimeError("Another mode request is pending") from error
        if not completed.wait(0.3):
            completed.set()
            raise RuntimeError("Controller did not acknowledge the mode request")
        return self.get_status()

    def process_mode_requests(self, now: float) -> None:
        try:
            mode, completed, deadline = self._mode_requests.get_nowait()
        except queue.Empty:
            return
        try:
            if not completed.is_set() and self.clock() <= deadline:
                self.set_mode(mode, now=now)
        finally:
            completed.set()

    def can_move(self, now=None) -> bool:
        now = self.clock() if now is None else now
        self.game_display.permission.update(now)
        return (self.running and self.connected and self.operation_mode == "manual" and self.position_confirmed
                and self.game_display.movement_allowed)

    def set_pulse(self, channel: int, pulse_us: float) -> bool:
        if not self.armed or not self.can_move():
            self.disable("Movement permission is no longer valid", revoke=False)
            return False
        try:
            self._outputs_disabled = False
            self.board.set_pulse(channel, pulse_us)
        except Exception:
            self.disable("Servo output failed")
            raise
        return True

    def disable(self, reason: str, *, now=None, revoke=True) -> None:
        now = self.clock() if now is None else now
        self.returning_home = False
        self.return_waypoints = []
        self.return_waypoint_index = 0
        self.return_mode = None
        self.escape_only = False
        self.home_required_after_stop = True
        was_armed = self.armed
        self.armed = False
        self.position_confirmed = False
        self.velocities = {servo.name: 0.0 for servo in self.settings.axis_servos}
        self.gripper_target = self.pulses["gripper"]
        self.last_stop_reason = reason
        try:
            if not self._outputs_disabled:
                self.board.disable_outputs()
                self._outputs_disabled = True
        finally:
            if revoke:
                self.game_display.cancel(now, reason)
            self.game_display.refresh(now, False)
        if was_armed:
            print(f"\nPWM disabled: {reason}")

    def confirm_position(self, state: ControllerState) -> bool:
        if (self.armed or not self.running or not self.connected or self.operation_mode != "manual" or not self.neutral(state)
                or self.game_display.emergency_stop_active or self.game_display.stop_button.is_pressed):
            return False
        self.position_confirmed = True
        self.gripper_target = self.pulses["gripper"]
        self.last_stop_reason = "Stored pulse pose confirmed; press green or white for permission"
        return True

    def reset_to_home(self) -> None:
        self.pulses = {
            servo.name: servo.start_pulse_us
            for servo in self.settings.axis_servos
        }

        self.pulses["gripper"] = (
            self.settings
            .gripper
            .start_pulse_us
        )

    def build_game_over_return_path(
        self,
    ) -> list[dict[str, float]]:
        current = tuple(
            self.pulses[name]
            for name in RETURN_AXES
        )

        home = tuple(
            next(
                servo.start_pulse_us
                for servo
                in self.settings.axis_servos
                if servo.name == name
            )
            for name in RETURN_AXES
        )

        def distance(
            first: tuple[float, ...],
            second: tuple[float, ...],
        ) -> float:
            return math.sqrt(
                sum(
                    (left - right) ** 2
                    for left, right
                    in zip(first, second)
                )
            )

        def planar_distance(
            first: tuple[float, ...],
            second: tuple[float, ...],
        ) -> float:
            return distance(
                first[1:],
                second[1:],
            )

        home_distance = planar_distance(
            current,
            home,
        )

        if (
            home_distance
            <= DIRECT_HOME_MAX_DISTANCE_US
        ):
            print(
                "\nArm shape is already near home. "
                f"Planar distance="
                f"{home_distance:.1f} us."
            )

            return [
                dict(
                    zip(
                        RETURN_AXES,
                        home,
                    )
                )
            ]

        for (
            region_name,
            limits,
        ) in MEASURED_DIRECT_HOME_REGIONS:
            inside_region = all(
                limits[name][0]
                <= self.pulses[name]
                <= limits[name][1]
                for name in RETURN_AXES
            )

            if inside_region:
                print(
                    "\nMeasured direct-home region "
                    f"selected: {region_name}."
                )

                return [
                    dict(
                        zip(
                            RETURN_AXES,
                            home,
                        )
                    )
                ]

        candidates = []

        for (
            route_name,
            anchor,
            waypoints,
        ) in SAFE_RETURN_ROUTES:
            route_points = (
                (anchor,) + waypoints
            )

            nearest_index = min(
                range(len(route_points)),
                key=lambda index: planar_distance(
                    current,
                    route_points[index],
                ),
            )

            nearest_point = (
                route_points[nearest_index]
            )

            candidates.append(
                (
                    planar_distance(
                        current,
                        nearest_point,
                    ),
                    route_name,
                    waypoints,
                    nearest_index,
                    nearest_point,
                )
            )

        (
            nearest_distance,
            route_name,
            waypoints,
            nearest_index,
            nearest_point,
        ) = min(
            candidates,
            key=lambda candidate: candidate[0],
        )

        if (
            nearest_distance
            > SAFE_ROUTE_MATCH_MAX_DISTANCE_US
        ):
            print(
                "\nNO SAFE RETURN ROUTE: "
                f"closest={route_name}, "
                f"planar distance="
                f"{nearest_distance:.1f} us."
            )

            return []

        print(
            "\nSafe return route selected: "
            f"{route_name}; "
            f"planar distance="
            f"{nearest_distance:.1f} us; "
            f"path point={nearest_index}."
        )

        base_offset = (
            current[0] - nearest_point[0]
        )

        result: list[
            dict[str, float]
        ] = []

        for recorded in waypoints[
            nearest_index:
        ]:
            shifted = list(
                map(float, recorded)
            )

            shifted[0] += base_offset

            shifted[0] = min(
                shifted[0],
                GLASS_DOOR_BASE_MAX_US,
            )

            result.append(
                dict(
                    zip(
                        RETURN_AXES,
                        shifted,
                    )
                )
            )

        result.append(
            dict(
                zip(
                    RETURN_AXES,
                    home,
                )
            )
        )

        return result

    def enable(self) -> bool:
        if not self.can_move():
            self.disable("No permission, verified pose or manual operating mode")
            return False
        self.armed = True
        self.velocities = {servo.name: 0.0 for servo in self.settings.axis_servos}
        self.gripper_target = self.pulses["gripper"]
        self.escape_only = self.glass_guard_active()
        try:
            for servo in self.settings.axis_servos:
                if not self.set_pulse(servo.channel, self.pulses[servo.name]):
                    return False
            if not self.set_pulse(self.settings.gripper.channel, self.pulses["gripper"]):
                return False
        except Exception:
            self.disable("Could not enable servo outputs")
            raise
        self.home_required_after_stop = False
        self.last_stop_reason = "Manual movement permission active"
        self.game_display.refresh(self.clock(), True)
        return True

    def glass_guard_active(self) -> bool:
        return (
            self.pulses["base"]
            >= GLASS_GUARD_BASE_START_US
            and self.pulses["shoulder"]
            >= GLASS_GUARD_SHOULDER_START_US
            and self.pulses["elbow"]
            <= GLASS_GUARD_ELBOW_MIN_US
        )

    def near_home_arm_shape(self) -> bool:
        home = {
            servo.name: servo.start_pulse_us
            for servo in self.settings.axis_servos
        }

        planar_distance = math.sqrt(
            sum(
                (
                    self.pulses[name]
                    - home[name]
                ) ** 2
                for name in (
                    "shoulder",
                    "elbow",
                    "wrist",
                )
            )
        )

        return (
            planar_distance
            <= DIRECT_HOME_MAX_DISTANCE_US
        )

    def toggle_enabled(self) -> None:
        if self.armed:
            self.disable("D-pad UP pressed")
        elif self.game_display.movement_allowed:
            self.enable()

    def apply_manual_safety_limit(
        self,
        servo_name: str,
        candidate: float,
    ) -> float:
        current = self.pulses

        if servo_name == "base":
            candidate = min(
                candidate,
                GLASS_DOOR_BASE_MAX_US,
            )

            if (
                current["shoulder"]
                >= GLASS_GUARD_SHOULDER_START_US
                and current["elbow"]
                <= GLASS_GUARD_ELBOW_MIN_US
            ):
                candidate = min(
                    candidate,
                    current["base"],
                )

        elif servo_name == "shoulder":
            if (
                current["base"]
                >= GLASS_GUARD_BASE_START_US
                and current["elbow"]
                <= GLASS_GUARD_ELBOW_MIN_US
            ):
                candidate = min(
                    candidate,
                    current["shoulder"],
                )

        elif servo_name == "elbow":
            if (
                current["base"]
                >= GLASS_GUARD_BASE_START_US
                and current["shoulder"]
                >= GLASS_GUARD_SHOULDER_START_US
            ):
                candidate = max(
                    candidate,
                    current["elbow"],
                )

        return candidate

    def update_motion(self, state: ControllerState, elapsed: float) -> None:
        if not self.armed or not self.can_move():
            if self.armed:
                self.disable("Movement permission expired", revoke=False)
            return
        elapsed = max(0.0, min(elapsed, 2.0 / self.settings.loop_hz))
        for servo in self.settings.axis_servos:
            target_velocity = (axis_amount(getattr(state, servo.axis), self.settings.dead_zone)
                               * servo.invert * servo.speed_us_per_second)
            step = servo.speed_us_per_second * elapsed / self.settings.motion_ramp_seconds
            velocity = self.velocities[servo.name]
            velocity += max(-step, min(step, target_velocity - velocity))
            self.velocities[servo.name] = velocity
            pulse = self.pulses[servo.name] + velocity * elapsed
            pulse = max(servo.minimum_pulse_us, min(servo.maximum_pulse_us, pulse))
            if self.escape_only:
                if servo.name in ("base", "shoulder"):
                    pulse = min(pulse, self.pulses[servo.name])
                elif servo.name == "elbow":
                    pulse = max(pulse, self.pulses[servo.name])
                else:
                    pulse = self.pulses[servo.name]
            pulse = self.apply_manual_safety_limit(servo.name, pulse)
            if pulse != self.pulses[servo.name]:
                if not self.set_pulse(servo.channel, pulse):
                    return
                self.pulses[servo.name] = pulse
        if self.escape_only:
            if self.near_home_arm_shape():
                self.disable("Escape position reached; confirm pose and request fresh permission")
            return
        if state.pressed(PSB_R1) != state.pressed(PSB_L1):
            self.gripper_target = (self.settings.gripper.open_pulse_us if state.pressed(PSB_R1)
                                   else self.settings.gripper.closed_pulse_us)
        difference = self.gripper_target - self.pulses["gripper"]
        step = self.settings.gripper.speed_us_per_second * elapsed
        pulse = self.pulses["gripper"] + max(-step, min(step, difference))
        if pulse != self.pulses["gripper"]:
            if not self.set_pulse(self.settings.gripper.channel, pulse):
                return
            self.pulses["gripper"] = pulse

    def control_step(self, state: ControllerState | None, now: float, elapsed: float) -> None:
        self.process_mode_requests(now)
        event = self.game_display.update(now, self.armed)
        if event in ("game_stopped", "emergency_stop", "game_over"):
            self.disable(event, now=now, revoke=False)
        if self.game_display.emergency_stop_active or self.game_display.stop_button.is_pressed:
            if self.armed or self.position_confirmed:
                self.disable("Stop input active", now=now)
            if state is not None:
                self.previous_buttons = state.buttons
            return
        if state is None:
            # Drop permission even during the packet timeout grace period.
            if self.armed or self.position_confirmed or self.game_display.movement_allowed:
                self.disable("Joystick input unavailable", now=now)
            self.previous_buttons = 0xFFFF
            return
        pressed_edges = state.buttons & ~self.previous_buttons
        self.previous_buttons = state.buttons
        if state.pressed(PSB_DISABLE):
            self.disable("D-pad DOWN pressed", now=now)
            return
        if pressed_edges & PSB_MODE:
            self.set_mode("automatic" if self.operation_mode == "manual" else "manual", now=now)
            return
        if pressed_edges & PSB_CONFIRM:
            self.confirm_position(state)
        if event in ("game_started", "setup_started"):
            if self.connected and self.neutral(state) and self.position_confirmed:
                self.enable()
            else:
                self.disable("Centre sticks, confirm the actual pose with START, then request fresh permission", now=now)
        elif pressed_edges & PSB_TOGGLE:
            self.toggle_enabled()
        if self.armed:
            self.update_motion(state, elapsed)
        self.game_display.refresh(now, self.armed)

    def display_status(
        self,
        state: ControllerState,
    ) -> None:
        if self.escape_only:
            mode = "ESCAPE"
        else:
            mode = (
                "ARMED"
                if self.armed
                else "DISABLED"
            )

        mode = f"{self.operation_mode.upper()} {mode}"
        pulse_text = " ".join(
            f"{name}={pulse:.0f}"
            for name, pulse
            in self.pulses.items()
        )

        xyz_text = "XYZ=unavailable"

        if self.kinematics is not None:
            try:
                (
                    x_mm,
                    y_mm,
                    z_mm,
                ) = self.kinematics.gripper_xyz(
                    self.pulses,
                    self.settings,
                )

                xyz_text = (
                    f"XYZ~=({x_mm:.1f},"
                    f"{y_mm:.1f},"
                    f"{z_mm:.1f})mm"
                )

            except (
                KeyError,
                ValueError,
                ZeroDivisionError,
            ) as error:
                xyz_text = (
                    f"XYZ=error:{error}"
                )

        print_controller_state(
            state,
            f"{mode} {pulse_text} {xyz_text}",
        )

    def run(self) -> None:
        print("START confirms actual stored pose; SELECT switches manual/automatic.")
        print("Green grants timed permission; white setup (if wired); red/D-pad DOWN revoke.")
        print("Automatic movement is unavailable; no return or gripper opening occurs after a stop.")
        if self.settings.control_api_enabled:
            from control_api import start_control_api
            try:
                self.api_server = start_control_api(self.get_status, self.request_mode,
                                                    port=self.settings.control_api_port)
            except OSError as error:
                print(f"Mode/status API unavailable: {error}")
        loop_interval = 1.0 / self.settings.loop_hz
        try:
            while self.running:
                now = self.clock()
                elapsed = min(0.1, max(0.0, now - self.last_loop))
                self.last_loop = now
                state = None
                try:
                    if not self.connected and now >= self.next_retry:
                        self.next_retry = now + self.settings.reconnect_interval_seconds
                        self.connected = self.controller.begin()
                        self.previous_buttons = 0xFFFF
                        self.position_confirmed = False
                    if self.connected:
                        state = self.controller.read()
                except OSError as error:
                    print(f"Joystick read failed: {error}")
                if state is None:
                    if now - self.last_good_read >= self.settings.disconnect_timeout_seconds:
                        self.connected = False
                else:
                    self.last_good_read = now
                self.control_step(state, now, elapsed)
                if state is not None and now >= self.next_status:
                    self.next_status = now + 1.0 / self.settings.status_hz
                    self.display_status(state)
                time.sleep(loop_interval)
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.running = False
        try:
            self.disable("Controller shutdown")
        finally:
            try:
                self.board.close()
            finally:
                try:
                    if self.api_server is not None:
                        self.api_server.shutdown()
                        self.api_server.server_close()
                        self.api_server = None
                finally:
                    try:
                        self.game_display.close()
                    finally:
                        self.controller.close()
        print("All configured PCA9685 outputs are OFF.")


def run_simulation(settings: Settings) -> None:
    now = time.monotonic()
    button = lambda: SimpleNamespace(is_pressed=False, close=lambda: None)
    light = SimpleNamespace(value=False, close=lambda: None)
    light.on = lambda: setattr(light, "value", True)
    light.off = lambda: setattr(light, "value", False)
    devices = SimpleNamespace(
        start_button=button(), stop_button=button(), setup_button=button(),
        emergency_stop_button=button(), game_light=light,
        lcd=SimpleNamespace(cursor_pos=(0, 0), write_string=lambda text: None,
                            clear=lambda: None, close=lambda **kwargs: None),
    )
    devices.emergency_stop_button.is_pressed = True
    writes = []
    board = SimpleNamespace(set_pulse=lambda channel, pulse: writes.append((channel, pulse)),
                            disable_outputs=lambda: None, close=lambda: None)
    joystick = SimpleNamespace(close=lambda: None)
    display = GameDisplay(settings.permission_seconds, notice_seconds=settings.end_message_seconds,
                          emergency_stop_healthy_pressed=True, devices=devices)
    controller = ManualController(settings, controller=joystick, board=board,
                                  game_display=display, simulated=True, clock=lambda: now)
    controller.connected = True
    axes = dict(left_x=128, left_y=128, right_x=128, right_y=128)
    print("SIMULATION ONLY: no GPIO, LCD, gamepad or PCA9685 is opened.")
    print("Commands: confirm, green, setup, red, manual, automatic, up, down,")
    print("  stick left_x|left_y|right_x|right_y 0..255, open, grip, neutral,")
    print("  tick SECONDS, disconnect, reconnect, estop, reset, status, quit")

    def step(buttons=0, elapsed=0.02):
        state = ControllerState(**axes, buttons=buttons) if controller.connected else None
        controller.control_step(state, now, elapsed)

    try:
        for line in sys.stdin:
            args = line.strip().split()
            if not args:
                continue
            command = args[0].lower()
            try:
                if command in ("quit", "q"):
                    break
                elif command in ("green", "setup", "red"):
                    button_input = {"green": devices.start_button, "setup": devices.setup_button,
                                    "red": devices.stop_button}[command]
                    button_input.is_pressed = True
                    step()
                    button_input.is_pressed = False
                    step(elapsed=0)
                elif command in ("manual", "automatic"):
                    controller.set_mode(command, now=now)
                elif command in ("confirm", "up", "down", "open", "grip"):
                    step({"confirm": PSB_CONFIRM, "up": PSB_TOGGLE, "down": PSB_DISABLE,
                          "open": PSB_R1, "grip": PSB_L1}[command])
                    step(elapsed=0)
                elif command == "stick":
                    if len(args) != 3 or args[1] not in axes or not 0 <= int(args[2]) <= 255:
                        raise ValueError("Use stick AXIS VALUE (0..255)")
                    axes[args[1]] = int(args[2])
                    step()
                elif command == "neutral":
                    axes = {name: 128 for name in axes}
                    step()
                elif command == "tick":
                    seconds = float(args[1])
                    if not math.isfinite(seconds) or not 0 <= seconds <= 3600:
                        raise ValueError("Tick must be finite and from 0 to 3600 seconds")
                    remaining = seconds
                    target_time = now + seconds
                    while remaining > 1e-9:
                        elapsed = min(1.0 / settings.loop_hz, remaining)
                        now += elapsed
                        step(elapsed=elapsed)
                        remaining -= elapsed
                    now = target_time
                    step(elapsed=0)
                elif command in ("disconnect", "reconnect"):
                    controller.connected = command == "reconnect"
                    controller.disable("Simulated joystick connection changed", now=now)
                elif command in ("estop", "reset"):
                    devices.emergency_stop_button.is_pressed = command == "reset"
                    step()
                elif command != "status":
                    raise ValueError("Unknown command")
                print(json.dumps({**controller.get_status(), "led": light.value,
                                  "simulated_output_writes": len(writes), "pulses_us": controller.pulses}))
            except (ValueError, IndexError) as error:
                print(f"Invalid simulation command: {error}")
    finally:
        controller.close()


def main() -> None:
    args = parse_args()

    default_config = (
        Path(__file__).resolve().parent
        / "config"
        / "ps2_controller.toml"
    )

    config_path = (
        args.config.resolve()
        if args.config is not None
        else default_config
    )

    settings = load_settings(
        config_path
    )

    if args.simulate:
        run_simulation(settings)
        return

    if args.diagnose:
        run_diagnostics(
            settings
        )
        return

    controller = ManualController(
        settings
    )

    def request_stop(
        _signum: int,
        _frame: object,
    ) -> None:
        controller.running = False

    signal.signal(
        signal.SIGTERM,
        request_stop,
    )

    controller.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(
            f"ERROR: {error}",
            file=sys.stderr,
        )
        raise SystemExit(1)
