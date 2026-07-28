from __future__ import annotations

import unittest

import config


class ServoConfigurationTests(unittest.TestCase):
    def test_legacy_controller_values_are_loaded_from_toml(self) -> None:
        self.assertEqual(config.PCA9685_FREQUENCY_HZ, 50)
        self.assertEqual(config.PCA9685_RESOLUTION_COUNTS, 4096)
        self.assertEqual(config.SERVO_PULSE_MIN, 150)
        self.assertEqual(config.SERVO_PULSE_MAX, 600)
        self.assertEqual(
            set(config.SERVOS),
            {"base", "shoulder", "elbow", "wrist"},
        )

    def test_channels_are_unique(self) -> None:
        channels = [servo.channel for servo in config.SERVOS.values()]
        channels.append(config.GRIPPER_CHANNEL)
        self.assertEqual(len(channels), len(set(channels)))

    def test_canonical_joint_names_are_available(self) -> None:
        joint_names = {servo.joint_name for servo in config.SERVOS.values()}
        joint_names.add(config.GRIPPER_SERVO.joint_name)
        self.assertEqual(
            joint_names,
            {
                "J1_base",
                "J2_shoulder",
                "J3_elbow",
                "J4_wrist",
                "J5_gripper",
            },
        )


if __name__ == "__main__":
    unittest.main()
