"""Interactive hardware keyboard control with exact PWM readout and recording."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import termios
import tty
from typing import Protocol

import config
from position_recorder import PositionRecorder
from pwm_output import PwmOutput, build_pwm_output


MOVEMENT_KEYS = {
    "a": ("base", -1),
    "d": ("base", 1),
    "w": ("shoulder", 1),
    "s": ("shoulder", -1),
    "i": ("elbow", 1),
    "k": ("elbow", -1),
    "j": ("wrist", -1),
    "l": ("wrist", 1),
}


class ServoBoardProtocol(Protocol):
    def set_servo(self, servo: config.ServoConfig, angle: int) -> PwmOutput:
        ...

    def close(self) -> None:
        ...


class KeyboardController:
    def __init__(
        self,
        board: ServoBoardProtocol | None = None,
        recorder: PositionRecorder | None = None,
    ) -> None:
        self.board = board
        self.recorder = recorder
        self.positions = {
            name: servo.initial_angle
            for name, servo in config.SERVOS.items()
        }
        self.gripper_angle = config.GRIPPER_R1_ANGLE
        self.outputs: dict[str, PwmOutput] = {}
        self.status = "No hardware command sent yet."

    @staticmethod
    def _preview(servo: config.ServoConfig, angle: int) -> PwmOutput:
        return build_pwm_output(
            joint_name=servo.joint_name,
            channel=servo.channel,
            command_angle_deg=angle,
            count_min=config.SERVO_PULSE_MIN,
            count_max=config.SERVO_PULSE_MAX,
            frequency_hz=config.PCA9685_FREQUENCY_HZ,
            resolution_counts=config.PCA9685_RESOLUTION_COUNTS,
        )

    def _send(self, servo: config.ServoConfig, angle: int) -> PwmOutput:
        output = (
            self.board.set_servo(servo, angle)
            if self.board is not None
            else self._preview(servo, angle)
        )
        self.outputs[servo.joint_name] = output
        self.status = (
            f"Sent {servo.joint_name}: command={angle}, "
            f"count={output.pulse_count}, pulse={output.pulse_us:.6f} us"
        )
        return output

    def send_initial_position(self) -> None:
        """Send all configured startup commands and establish a complete snapshot."""
        for controller_name, servo in config.SERVOS.items():
            self._send(servo, self.positions[controller_name])
        self._send(config.GRIPPER_SERVO, self.gripper_angle)
        self.status = "Configured startup position sent to all five servos."

    def handle_key(self, key: str) -> bool:
        key = key.lower()

        if key == "q":
            return False

        if key in MOVEMENT_KEYS:
            name, direction = MOVEMENT_KEYS[key]
            servo = config.SERVOS[name]
            new_angle = self.positions[name] + direction * config.KEYBOARD_STEP_DEGREES
            bounded_angle = max(
                servo.minimum_angle,
                min(servo.maximum_angle, new_angle),
            )
            if bounded_angle == self.positions[name]:
                self.status = f"{servo.joint_name} is at its configured command limit."
                return True
            self.positions[name] = bounded_angle
            self._send(servo, bounded_angle)

        elif key == "o":
            self.gripper_angle = config.GRIPPER_R1_ANGLE
            self._send(config.GRIPPER_SERVO, self.gripper_angle)

        elif key == "c":
            self.gripper_angle = config.GRIPPER_L1_ANGLE
            self._send(config.GRIPPER_SERVO, self.gripper_angle)

        elif key == "r":
            if self.recorder is None:
                self.status = "Recording is not configured."
            elif len(self.outputs) != 5:
                self.status = "Send the complete startup position before recording."
            else:
                name = self.recorder.record(
                    self.outputs,
                    count_min=config.SERVO_PULSE_MIN,
                    count_max=config.SERVO_PULSE_MAX,
                )
                self.status = (
                    f"Recorded {name} to {self.recorder.csv_path} and "
                    f"{self.recorder.json_path}."
                )

        return True

    @staticmethod
    def _format_output(output: PwmOutput) -> str:
        warning = ""
        if not config.SERVO_PULSE_MIN <= output.pulse_count <= config.SERVO_PULSE_MAX:
            warning = (
                "  WARNING: outside legacy "
                f"{config.SERVO_PULSE_MIN}-{config.SERVO_PULSE_MAX} endpoint"
            )
        return (
            f"cmd={output.command_angle_deg:>4}  "
            f"count={output.pulse_count:>4}  "
            f"pulse={output.pulse_us:>11.6f} us  "
            f"duty={output.duty_cycle:>5}{warning}"
        )

    def display(self) -> None:
        print("\033[2J\033[H", end="")
        print("Robot keyboard control and PWM position recorder")
        print("================================================")
        print("a/d  base -/+        w/s  shoulder +/-")
        print("i/k  elbow +/-       j/l  wrist -/+")
        print("o/c  gripper open/close")
        print("r    record the current five-joint position")
        print("q    quit keyboard input")
        print()
        print(
            "Displayed command values are legacy controller units, not yet "
            "physically calibrated joint angles."
        )
        print("The pulse_us value is calculated from the exact integer PCA9685 count.")
        print()

        rows = [
            ("Base", config.SERVOS["base"], self.positions["base"]),
            ("Shoulder", config.SERVOS["shoulder"], self.positions["shoulder"]),
            ("Elbow", config.SERVOS["elbow"], self.positions["elbow"]),
            ("Wrist", config.SERVOS["wrist"], self.positions["wrist"]),
            ("Gripper", config.GRIPPER_SERVO, self.gripper_angle),
        ]
        for label, servo, angle in rows:
            output = self.outputs.get(servo.joint_name) or self._preview(servo, angle)
            print(
                f"{label:<9} {servo.joint_name:<13} ch={servo.channel:>2}  "
                f"{self._format_output(output)}"
            )

        print()
        print(f"Status: {self.status}")
        if self.recorder is not None:
            print(f"Record file: {self.recorder.csv_path}")
        print()
        print("Use the physical emergency stop to cut servo power immediately.", flush=True)

    def run(self) -> None:
        file_descriptor = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_descriptor)

        try:
            tty.setcbreak(file_descriptor)
            self.display()
            while True:
                key = os.read(file_descriptor, 1).decode("utf-8", errors="ignore")
                if not key:
                    continue
                if not self.handle_key(key):
                    break
                self.display()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(
                file_descriptor,
                termios.TCSADRAIN,
                old_settings,
            )
            print("\033[2J\033[H", end="")
            print("Keyboard control stopped at the last commanded position.")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Control the current Robot-Controller servo mapping from the keyboard, "
            "show exact PCA9685 output values, and record fixed positions."
        )
    )
    parser.add_argument(
        "--record-file",
        type=Path,
        default=Path("calibration/recorded_positions.csv"),
        help="CSV output; a JSON file with the same stem is also written",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the typed startup confirmation",
    )
    return parser


def main() -> None:
    args = _build_parser().parse_args()

    print("This program will send the configured initial position to all five servos.")
    print("Remove objects, support the arm if needed, and keep the E-stop accessible.")
    print("Current configured startup commands:")
    for name, servo in config.SERVOS.items():
        print(f"  {servo.joint_name}: {servo.initial_angle} on channel {servo.channel}")
    print(
        f"  {config.GRIPPER_SERVO.joint_name}: "
        f"{config.GRIPPER_R1_ANGLE} on channel {config.GRIPPER_SERVO.channel}"
    )

    if not args.yes:
        confirmation = input('Type MOVE to continue: ').strip()
        if confirmation != "MOVE":
            print("No command was sent.")
            return

    # Importing hardware is delayed so unit tests and --help work without Pi modules.
    from hardware import ServoBoard

    try:
        board = ServoBoard()
    except Exception as error:
        print()
        print("Hardware initialization failed:")
        print(error)
        raise SystemExit(2) from None

    controller = KeyboardController(
        board=board,
        recorder=PositionRecorder(args.record_file),
    )
    try:
        controller.send_initial_position()
        controller.run()
    finally:
        board.close()


if __name__ == "__main__":
    main()
