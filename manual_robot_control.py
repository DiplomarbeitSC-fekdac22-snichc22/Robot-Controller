#!/usr/bin/env python3
"""Live keyboard control for the Raspberry Pi robot arm.

The pulse starting points and directions are based on the physical tests made
on the robot.  The program starts with every PCA9685 output disabled and only
activates a joint after its movement key is pressed.
"""

from __future__ import annotations

import argparse
import atexit
from dataclasses import dataclass
import os
import signal
import sys
import termios
import time
import tty
from typing import Protocol


PCA9685_ADDRESS = 0x40
PWM_FREQUENCY_HZ = 50
MOVEMENT_DELAY_SECONDS = 0.08


@dataclass(frozen=True)
class Joint:
    name: str
    channel: int
    start_us: int
    minimum_us: int
    maximum_us: int
    step_us: int


# Directions confirmed on the real robot:
#   base:     higher pulse = left
#   shoulder: higher pulse = toward the objects
#   elbow:    lower pulse = extend farther
#   wrist:    higher pulse = forward
#   gripper:  lower pulse = open
JOINTS = {
    "base": Joint("Base", 0, 1830, 750, 2300, 20),
    "shoulder": Joint("Shoulder", 2, 1550, 1200, 1800, 10),
    "elbow": Joint("Elbow", 4, 1520, 1200, 1800, 10),
    "wrist": Joint("Wrist", 6, 1500, 1200, 1800, 10),
    # The gripper is currently closed, so its session start is 1275 us.
    "gripper": Joint("Gripper", 8, 1275, 700, 1275, 25),
}


# key: (joint, pulse direction, human-readable movement)
MOVEMENTS = {
    "a": ("base", +1, "left"),
    "d": ("base", -1, "right"),
    "w": ("shoulder", -1, "up / away"),
    "s": ("shoulder", +1, "down / toward objects"),
    "i": ("elbow", -1, "extend"),
    "k": ("elbow", +1, "bend"),
    "j": ("wrist", -1, "back / up"),
    "l": ("wrist", +1, "forward / down"),
    "o": ("gripper", -1, "open"),
    "c": ("gripper", +1, "close"),
}


class ServoOutput(Protocol):
    def set_pulse_us(self, channel: int, pulse_us: int) -> None: ...

    def disable_all(self) -> None: ...

    def close(self) -> None: ...


def pulse_to_duty_cycle(pulse_us: int) -> int:
    """Convert microseconds at 50 Hz to the PCA9685 driver's 16-bit duty."""
    count = round(pulse_us * PWM_FREQUENCY_HZ * 4096 / 1_000_000)
    return count << 4


class PcaServoOutput:
    """Small hardware adapter with no LCD, buttons, gamepad, or relay imports."""

    def __init__(self) -> None:
        # Keep Raspberry Pi-only imports out of module import so tests can run
        # on another computer.
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(self._i2c, address=PCA9685_ADDRESS)
        self._pca.frequency = PWM_FREQUENCY_HZ
        self._closed = False
        self.disable_all()

    def set_pulse_us(self, channel: int, pulse_us: int) -> None:
        if self._closed:
            raise RuntimeError("Servo output is already closed")
        self._pca.channels[channel].duty_cycle = pulse_to_duty_cycle(pulse_us)

    def disable_all(self) -> None:
        if self._closed:
            return
        for channel in range(16):
            self._pca.channels[channel].duty_cycle = 0

    def close(self) -> None:
        if self._closed:
            return
        self.disable_all()
        self._closed = True
        self._pca.deinit()
        self._i2c.deinit()


class DryRunOutput:
    """Output used by --dry-run and automated tests."""

    def __init__(self) -> None:
        self.commands: list[tuple[int, int]] = []
        self.disable_count = 0

    def set_pulse_us(self, channel: int, pulse_us: int) -> None:
        self.commands.append((channel, pulse_us))

    def disable_all(self) -> None:
        self.disable_count += 1

    def close(self) -> None:
        self.disable_all()


class KeyboardController:
    def __init__(self, output: ServoOutput) -> None:
        self.output = output
        self.pulses = {name: joint.start_us for name, joint in JOINTS.items()}
        self.active: set[str] = set()
        self.message = "Outputs are OFF. Press a movement key to activate one joint."

    def handle_key(self, key: str) -> bool:
        key = key.lower()

        if key == "q":
            self.output.disable_all()
            self.active.clear()
            return False

        if key in (" ", "x"):
            self.output.disable_all()
            self.active.clear()
            self.message = "EMERGENCY OFF: all PCA9685 outputs disabled."
            return True

        movement = MOVEMENTS.get(key)
        if movement is None:
            self.message = f"Unknown key: {key!r}"
            return True

        name, direction, label = movement
        joint = JOINTS[name]
        requested = self.pulses[name] + direction * joint.step_us
        pulse = max(joint.minimum_us, min(joint.maximum_us, requested))
        self.pulses[name] = pulse
        self.output.set_pulse_us(joint.channel, pulse)
        self.active.add(name)

        if pulse == requested:
            self.message = f"{joint.name}: {label} -> {pulse} us"
        else:
            self.message = f"{joint.name}: {pulse} us limit reached"

        time.sleep(MOVEMENT_DELAY_SECONDS)
        return True

    def display(self) -> None:
        print("\033[2J\033[H", end="")
        print("ROBOT ARM — LIVE MANUAL CONTROL")
        print("================================")
        print("A / D   base left / right")
        print("W / S   shoulder up / down")
        print("I / K   elbow extend / bend")
        print("J / L   wrist back-up / forward-down")
        print("O / C   gripper open / close")
        print("SPACE/X emergency outputs off")
        print("Q       outputs off and quit")
        print()

        for name, joint in JOINTS.items():
            state = "HOLDING" if name in self.active else "off"
            print(
                f"{joint.name:<9} CH{joint.channel:<2} "
                f"{self.pulses[name]:>4} us  [{state}]"
            )

        print()
        print(self.message, flush=True)

    def run(self) -> None:
        file_descriptor = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_descriptor)

        try:
            tty.setcbreak(file_descriptor)
            self.display()

            while True:
                key = os.read(file_descriptor, 1).decode("utf-8", errors="ignore")
                if key and not self.handle_key(key):
                    break
                self.display()
        except KeyboardInterrupt:
            pass
        finally:
            self.output.disable_all()
            self.active.clear()
            termios.tcsetattr(
                file_descriptor,
                termios.TCSADRAIN,
                old_settings,
            )
            print("\033[2J\033[H", end="")
            print("Manual control stopped. All PCA9685 outputs are OFF.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="test the keyboard UI without accessing Raspberry Pi hardware",
    )
    arguments = parser.parse_args()

    output: ServoOutput
    if arguments.dry_run:
        output = DryRunOutput()
    else:
        output = PcaServoOutput()

    atexit.register(output.close)

    def stop_on_signal(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_on_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, stop_on_signal)

    KeyboardController(output).run()


if __name__ == "__main__":
    main()
