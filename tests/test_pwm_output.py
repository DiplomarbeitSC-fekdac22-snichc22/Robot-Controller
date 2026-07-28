from __future__ import annotations

import unittest

from pwm_output import (
    angle_to_count,
    build_pwm_output,
    count_to_duty_cycle,
    count_to_microseconds,
)


class PwmOutputTests(unittest.TestCase):
    def test_legacy_mapping_matches_known_points(self) -> None:
        self.assertEqual(angle_to_count(0, count_min=150, count_max=600), 150)
        self.assertEqual(angle_to_count(90, count_min=150, count_max=600), 375)
        self.assertEqual(angle_to_count(180, count_min=150, count_max=600), 600)
        self.assertEqual(angle_to_count(230, count_min=150, count_max=600), 725)

    def test_count_to_microseconds_uses_real_period(self) -> None:
        self.assertAlmostEqual(
            count_to_microseconds(150, frequency_hz=50, resolution_counts=4096),
            732.421875,
        )
        self.assertAlmostEqual(
            count_to_microseconds(600, frequency_hz=50, resolution_counts=4096),
            2929.6875,
        )

    def test_duty_cycle_is_exact_16_bit_expansion(self) -> None:
        self.assertEqual(count_to_duty_cycle(375, resolution_counts=4096), 6000)

    def test_output_contains_all_exact_values(self) -> None:
        output = build_pwm_output(
            joint_name="J1_base",
            channel=0,
            command_angle_deg=90,
            count_min=150,
            count_max=600,
            frequency_hz=50,
            resolution_counts=4096,
        )
        self.assertEqual(output.pulse_count, 375)
        self.assertAlmostEqual(output.pulse_us, 1831.0546875)
        self.assertEqual(output.duty_cycle, 6000)


if __name__ == "__main__":
    unittest.main()
