"""Pure PCA9685 PWM conversion helpers used by manual control and tests."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class PwmOutput:
    """Exact digital output produced for one servo command."""

    joint_name: str
    channel: int
    command_angle_deg: int
    pulse_count: int
    pulse_us: float
    duty_cycle: int

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def angle_to_count(
    angle_deg: int,
    *,
    count_min: int,
    count_max: int,
    mapping_angle_range_deg: int = 180,
) -> int:
    """Reproduce Arduino ``map(angle, 0, 180, min, max)`` integer behavior.

    The legacy Robot-Controller configuration expresses servo commands as integer
    values named as degrees. They are controller units until a physical angle
    calibration is completed. Values above 180 are intentionally extrapolated,
    matching the existing controller.
    """
    if mapping_angle_range_deg <= 0:
        raise ValueError("mapping_angle_range_deg must be positive")
    if count_min >= count_max:
        raise ValueError("count_min must be lower than count_max")

    # int() truncates toward zero, matching C/C++ integer division used by Arduino.
    offset = int(angle_deg * (count_max - count_min) / mapping_angle_range_deg)
    return count_min + offset


def count_to_microseconds(
    pulse_count: int,
    *,
    frequency_hz: float,
    resolution_counts: int,
) -> float:
    """Return the physical high-time represented by one PCA9685 count value."""
    if frequency_hz <= 0:
        raise ValueError("frequency_hz must be positive")
    if resolution_counts <= 0:
        raise ValueError("resolution_counts must be positive")
    return pulse_count * 1_000_000.0 / frequency_hz / resolution_counts


def count_to_duty_cycle(
    pulse_count: int,
    *,
    resolution_counts: int,
    driver_steps: int = 65_536,
) -> int:
    """Convert a PCA9685 count to CircuitPython's 16-bit duty-cycle value."""
    if resolution_counts <= 0:
        raise ValueError("resolution_counts must be positive")
    if driver_steps <= 0 or driver_steps % resolution_counts != 0:
        raise ValueError("driver_steps must be a positive multiple of resolution_counts")
    if not 0 <= pulse_count < resolution_counts:
        raise ValueError(
            f"PCA9685 count is out of range: {pulse_count}; expected 0-{resolution_counts - 1}"
        )
    return pulse_count * (driver_steps // resolution_counts)


def build_pwm_output(
    *,
    joint_name: str,
    channel: int,
    command_angle_deg: int,
    count_min: int,
    count_max: int,
    frequency_hz: float,
    resolution_counts: int,
) -> PwmOutput:
    """Calculate the exact count, high-time and duty value sent to hardware."""
    pulse_count = angle_to_count(
        command_angle_deg,
        count_min=count_min,
        count_max=count_max,
    )
    duty_cycle = count_to_duty_cycle(
        pulse_count,
        resolution_counts=resolution_counts,
    )
    return PwmOutput(
        joint_name=joint_name,
        channel=channel,
        command_angle_deg=command_angle_deg,
        pulse_count=pulse_count,
        pulse_us=count_to_microseconds(
            pulse_count,
            frequency_hz=frequency_hz,
            resolution_counts=resolution_counts,
        ),
        duty_cycle=duty_cycle,
    )
