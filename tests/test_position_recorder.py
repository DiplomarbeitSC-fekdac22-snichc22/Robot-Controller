from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path

from position_recorder import PositionRecorder
from pwm_output import PwmOutput


class PositionRecorderTests(unittest.TestCase):
    def test_records_csv_and_json_with_exact_pulse_values(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "positions.csv"
            recorder = PositionRecorder(path)
            outputs = {
                "J1_base": PwmOutput(
                    joint_name="J1_base",
                    channel=0,
                    command_angle_deg=90,
                    pulse_count=375,
                    pulse_us=1831.0546875,
                    duty_cycle=6000,
                )
            }

            name = recorder.record(outputs, count_min=150, count_max=600)

            self.assertEqual(name, "position_001")
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["pulse_count"], "375")
            self.assertEqual(rows[0]["pulse_us"], "1831.054687500")

            data = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
            joint = data["positions"]["position_001"]["joints"]["J1_base"]
            self.assertEqual(joint["pulse_count"], 375)
            self.assertAlmostEqual(joint["pulse_us"], 1831.0546875)


if __name__ == "__main__":
    unittest.main()
