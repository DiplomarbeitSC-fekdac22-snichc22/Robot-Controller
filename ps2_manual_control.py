from __future__ import annotations

import argparse
import math
import signal
import sys
import time
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ps2_controller import ControllerState, Ps2Controller
from game_display import GameDisplay


PSB_TOGGLE = 1 << 4
PSB_DISABLE = 1 << 6
PSB_L1 = 1 << 10
PSB_R1 = 1 << 11

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
        ),
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

        self._pca = PCA9685(
            self._i2c,
            address=settings.i2c_address,
        )

        self._pca.frequency = (
            settings.frequency_hz
        )

        self.disable_outputs()

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
        for channel in self._channels:
            self._pca.channels[
                channel
            ].duty_cycle = 0

    def close(self) -> None:
        self.disable_outputs()
        self._pca.deinit()
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
    ) -> None:
        self.settings = settings
        self.controller = Ps2Controller()

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
            self.board = PulseServoBoard(
                settings
            )
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

        self.kinematics = (
            load_kinematics_model()
        )

        try:
            self.game_display = GameDisplay(
                duration_seconds=30.0,
            )
        except Exception:
            self.board.close()
            self.controller.close()
            raise

        self.armed = False
        self.connected = False
        self.previous_buttons = 0
        self.last_good_read = 0.0
        self.next_retry = 0.0
        self.next_status = 0.0
        self.last_loop = time.monotonic()
        self.running = True

    def disable(
        self,
        reason: str,
    ) -> None:
        self.returning_home = False
        self.return_waypoints = []
        self.return_waypoint_index = 0
        self.return_mode = None
        self.escape_only = False

        self.home_required_after_stop = True

        was_armed = self.armed
        self.armed = False

        self.board.disable_outputs()

        if was_armed:
            print(
                f"\nPWM disabled: {reason}"
            )

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

    def start_game_over_return(
        self,
        now: float,
    ) -> None:
        if not self.armed:
            self.pulses["gripper"] = (
                self.settings
                .gripper
                .open_pulse_us
            )

            self.game_display.show_game_over_complete(
                home_reached=False,
            )

            print(
                "\nGame over, but PWM was already "
                "disabled. The arm could not "
                "physically return home."
            )
            return

        print(
            "\nGame over: opening gripper."
        )

        self.pulses["gripper"] = (
            self.settings
            .gripper
            .open_pulse_us
        )

        self.board.set_pulse(
            self.settings.gripper.channel,
            self.pulses["gripper"],
        )

        self.return_waypoints = (
            self.build_game_over_return_path()
        )

        self.return_waypoint_index = 0

        self.home_release_until = (
            now + GRIPPER_RELEASE_SECONDS
        )

        self.return_mode = "game_over"
        self.returning_home = True

    def update_game_over_return(
        self,
        now: float,
        elapsed: float,
    ) -> None:
        if (
            not self.returning_home
            or not self.armed
        ):
            return

        if now < self.home_release_until:
            return

        if not self.return_waypoints:
            self.returning_home = False
            self.return_mode = None

            self.game_display.show_game_over_complete(
                home_reached=False,
            )

            self.disable(
                "no recorded safe return route "
                "for this position"
            )

            return

        if (
            self.return_waypoint_index
            >= len(self.return_waypoints)
        ):
            return

        target = self.return_waypoints[
            self.return_waypoint_index
        ]

        waypoint_reached = True

        for servo in self.settings.axis_servos:
            current = self.pulses[
                servo.name
            ]

            target_pulse = target[
                servo.name
            ]

            speed = max(
                servo.speed_us_per_second,
                HOME_RETURN_MIN_SPEED_US_PER_SECOND,
            )

            maximum_step = (
                speed * elapsed
            )

            difference = (
                target_pulse - current
            )

            if abs(difference) <= maximum_step:
                new_pulse = target_pulse
            else:
                new_pulse = (
                    current
                    + math.copysign(
                        maximum_step,
                        difference,
                    )
                )

                waypoint_reached = False

            self.pulses[
                servo.name
            ] = new_pulse

            self.board.set_pulse(
                servo.channel,
                new_pulse,
            )

            if (
                abs(new_pulse - target_pulse)
                > 0.5
            ):
                waypoint_reached = False

        if not waypoint_reached:
            return

        self.return_waypoint_index += 1

        print(
            "\nReturn waypoint reached: "
            f"{self.return_waypoint_index}/"
            f"{len(self.return_waypoints)}"
        )

        if (
            self.return_waypoint_index
            < len(self.return_waypoints)
        ):
            return

        completed_mode = self.return_mode
        self.returning_home = False
        self.return_mode = None

        if (
            completed_mode
            == "emergency_recovery"
        ):
            print(
                "\nEmergency-stop recovery complete. "
                "The arm is at the TOML home position."
            )

            return

        self.game_display.show_game_over_complete(
            home_reached=True,
        )

        self.disable(
            "game over: gripper opened, "
            "shelf cleared and home position reached"
        )

    def enable(self) -> None:
        self.board.disable_outputs()

        return_path = (
            self.build_game_over_return_path()
        )

        if not return_path:
            if self.glass_guard_active():
                self.enable_escape_only()
                return

            print(
                "\nPWM remains disabled: there is no "
                "recorded safe return route for the "
                "current position."
            )

            return

        try:
            for servo in self.settings.axis_servos:
                self.board.set_pulse(
                    servo.channel,
                    self.pulses[servo.name],
                )

            self.pulses["gripper"] = (
                self.settings
                .gripper
                .open_pulse_us
            )

            self.board.set_pulse(
                self.settings.gripper.channel,
                self.pulses["gripper"],
            )

        except Exception:
            self.board.disable_outputs()
            raise

        self.home_required_after_stop = False
        self.armed = True
        self.escape_only = False
        self.return_waypoints = return_path
        self.return_waypoint_index = 0

        self.home_release_until = (
            time.monotonic()
            + GRIPPER_RELEASE_SECONDS
        )

        self.return_mode = (
            "emergency_recovery"
        )
        self.returning_home = True

        print(
            "\nPWM enabled. Emergency-stop recovery: "
            "opening the gripper and following a "
            "safe route to the TOML home position."
        )

    def glass_guard_active(self) -> bool:
        return (
            self.pulses["base"]
            >= GLASS_GUARD_BASE_START_US
            and self.pulses["shoulder"]
            >= GLASS_GUARD_SHOULDER_START_US
            and self.pulses["elbow"]
            <= GLASS_GUARD_ELBOW_MIN_US
        )

    def enable_escape_only(self) -> None:
        try:
            for servo in self.settings.axis_servos:
                self.board.set_pulse(
                    servo.channel,
                    self.pulses[servo.name],
                )

            self.board.set_pulse(
                self.settings.gripper.channel,
                self.pulses["gripper"],
            )

        except Exception:
            self.board.disable_outputs()
            raise

        self.armed = True
        self.escape_only = True
        self.returning_home = False
        self.return_mode = None
        self.return_waypoints = []
        self.return_waypoint_index = 0

        print(
            "\nESCAPE-ONLY MODE: this is a forbidden "
            "pose. Only movements that make base and "
            "shoulder numbers smaller or the elbow "
            "number larger are accepted. Wrist and "
            "gripper commands are blocked."
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
            self.disable(
                "D-pad UP pressed"
            )
        else:
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

    def update_motion(
        self,
        state: ControllerState,
        elapsed: float,
    ) -> None:
        for servo in self.settings.axis_servos:
            amount = axis_amount(
                getattr(
                    state,
                    servo.axis,
                ),
                self.settings.dead_zone,
            )

            if amount == 0.0:
                continue

            pulse = (
                self.pulses[servo.name]
                + amount
                * servo.invert
                * servo.speed_us_per_second
                * elapsed
            )

            pulse = max(
                servo.minimum_pulse_us,
                min(
                    servo.maximum_pulse_us,
                    pulse,
                ),
            )

            if self.escape_only:
                if servo.name == "base":
                    pulse = min(
                        pulse,
                        self.pulses["base"],
                    )

                elif servo.name == "shoulder":
                    pulse = min(
                        pulse,
                        self.pulses["shoulder"],
                    )

                elif servo.name == "elbow":
                    pulse = max(
                        pulse,
                        self.pulses["elbow"],
                    )

                else:
                    pulse = (
                        self.pulses[servo.name]
                    )

            pulse = (
                self.apply_manual_safety_limit(
                    servo.name,
                    pulse,
                )
            )

            self.pulses[
                servo.name
            ] = pulse

            self.board.set_pulse(
                servo.channel,
                pulse,
            )

        if self.escape_only:
            if self.near_home_arm_shape():
                self.disable(
                    "escape position reached"
                )

                print(
                    "\nThe dangerous arm shape has been "
                    "cleared. Press D-pad UP again to "
                    "follow the safe return to home."
                )

            return

        r1_pressed = state.pressed(
            PSB_R1
        )
        l1_pressed = state.pressed(
            PSB_L1
        )

        if (
            r1_pressed
            and not l1_pressed
        ):
            pulse = (
                self.settings
                .gripper
                .open_pulse_us
            )

            if (
                pulse
                != self.pulses["gripper"]
            ):
                self.pulses[
                    "gripper"
                ] = pulse

                self.board.set_pulse(
                    self.settings
                    .gripper
                    .channel,
                    pulse,
                )

        elif (
            l1_pressed
            and not r1_pressed
        ):
            pulse = (
                self.settings
                .gripper
                .closed_pulse_us
            )

            if (
                pulse
                != self.pulses["gripper"]
            ):
                self.pulses[
                    "gripper"
                ] = pulse

                self.board.set_pulse(
                    self.settings
                    .gripper
                    .channel,
                    pulse,
                )

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
        print("PS2 robot-arm control")
        print("=====================")
        print(
            "D-pad UP    enable/disable PWM"
        )
        print(
            "D-pad DOWN  emergency PWM disable"
        )
        print(
            "Left stick X/Y   base/shoulder"
        )
        print(
            "Right stick Y/X  elbow/wrist"
        )
        print(
            "R1 open gripper; L1 close gripper"
        )
        print(
            "Ctrl+C exits and disables every "
            "configured output"
        )
        print()
        print(
            "PWM is currently disabled."
        )

        loop_interval = (
            1.0 / self.settings.loop_hz
        )
        status_interval = (
            1.0 / self.settings.status_hz
        )

        try:
            while self.running:
                now = time.monotonic()

                elapsed = min(
                    0.100,
                    now - self.last_loop,
                )

                self.last_loop = now

                game_event = (
                    self.game_display.update(
                        now,
                        self.armed,
                    )
                )

                if (
                    game_event
                    == "game_started"
                ):
                    if not self.armed:
                        self.enable()

                elif (
                    game_event
                    == "game_stopped"
                ):
                    self.disable(
                        "red STOP button pressed"
                    )

                elif (
                    game_event
                    == "game_over"
                ):
                    self.start_game_over_return(
                        now
                    )

                if not self.connected:
                    self.disable(
                        "controller disconnected"
                    )

                    if now < self.next_retry:
                        time.sleep(
                            loop_interval
                        )
                        continue

                    self.next_retry = (
                        now
                        + self.settings
                        .reconnect_interval_seconds
                    )

                    try:
                        self.connected = (
                            self.controller.begin()
                        )
                    except OSError as error:
                        print(
                            f"\nPS2 SPI error: {error}"
                        )

                        self.connected = False

                    if not self.connected:
                        print(
                            "\rWaiting for "
                            "PS2 controller...",
                            end="",
                            flush=True,
                        )

                        time.sleep(
                            loop_interval
                        )
                        continue

                    self.previous_buttons = (
                        0xFFFF
                    )
                    self.last_good_read = now

                    print(
                        "\nPS2 controller connected. "
                        "PWM remains disabled."
                    )

                try:
                    state = (
                        self.controller.read()
                    )
                except OSError as error:
                    print(
                        f"\nPS2 read error: {error}"
                    )
                    state = None

                if state is None:
                    if (
                        now - self.last_good_read
                        >= self.settings
                        .disconnect_timeout_seconds
                    ):
                        self.connected = False

                        self.disable(
                            "controller timeout"
                        )

                    time.sleep(
                        loop_interval
                    )
                    continue

                self.last_good_read = now

                pressed_edges = (
                    state.buttons
                    & ~self.previous_buttons
                )

                self.previous_buttons = (
                    state.buttons
                )

                if (
                    pressed_edges
                    & PSB_DISABLE
                ):
                    self.disable(
                        "D-pad DOWN pressed"
                    )

                elif (
                    pressed_edges
                    & PSB_TOGGLE
                ):
                    self.toggle_enabled()

                if self.returning_home:
                    self.update_game_over_return(
                        now,
                        elapsed,
                    )

                elif self.armed:
                    self.update_motion(
                        state,
                        elapsed,
                    )

                if now >= self.next_status:
                    self.next_status = (
                        now + status_interval
                    )

                    self.display_status(
                        state
                    )

                time.sleep(
                    loop_interval
                )

        except KeyboardInterrupt:
            print()

        finally:
            self.close()

    def close(self) -> None:
        self.running = False

        try:
            self.board.close()
        finally:
            try:
                self.game_display.close()
            finally:
                self.controller.close()

        print(
            "All configured PCA9685 outputs are OFF."
        )


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