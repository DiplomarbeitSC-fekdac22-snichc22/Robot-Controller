from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import config
from keyboard_control import KeyboardController
from position_recorder import PositionRecorder
from pwm_output import build_pwm_output


class FakeServoBoard:
    def __init__(self) -> None:
        self.commands: list[tuple[str, int]] = []

    def set_servo(self, servo: config.ServoConfig, angle: int):
        self.commands.append((servo.joint_name, angle))
        return build_pwm_output(
            joint_name=servo.joint_name,
            channel=servo.channel,
            command_angle_deg=angle,
            count_min=config.SERVO_PULSE_MIN,
            count_max=config.SERVO_PULSE_MAX,
            frequency_hz=config.PCA9685_FREQUENCY_HZ,
            resolution_counts=config.PCA9685_RESOLUTION_COUNTS,
        )

    def close(self) -> None:
        pass


class KeyboardControllerTests(unittest.TestCase):
    def test_base_key_sends_hardware_command(self) -> None:
        board = FakeServoBoard()
        controller = KeyboardController(board=board)
        initial_angle = controller.positions["base"]

        controller.handle_key("d")

        self.assertEqual(
            controller.positions["base"],
            initial_angle + config.KEYBOARD_STEP_DEGREES,
        )
        self.assertEqual(
            board.commands[-1],
            ("J1_base", initial_angle + config.KEYBOARD_STEP_DEGREES),
        )
        self.assertIn("J1_base", controller.outputs)

    def test_initial_position_commands_all_five_servos(self) -> None:
        board = FakeServoBoard()
        controller = KeyboardController(board=board)

        controller.send_initial_position()

        self.assertEqual(len(board.commands), 5)
        self.assertEqual(set(controller.outputs), {
            "J1_base",
            "J2_shoulder",
            "J3_elbow",
            "J4_wrist",
            "J5_gripper",
        })

    def test_record_key_writes_current_snapshot(self) -> None:
        board = FakeServoBoard()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "positions.csv"
            controller = KeyboardController(
                board=board,
                recorder=PositionRecorder(path),
            )
            controller.send_initial_position()

            controller.handle_key("r")

            self.assertTrue(path.exists())
            self.assertTrue(path.with_suffix(".json").exists())
            self.assertIn("Recorded position_001", controller.status)


if __name__ == "__main__":
    unittest.main()
