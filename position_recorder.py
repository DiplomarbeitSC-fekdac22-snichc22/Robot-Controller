"""Persist manually selected robot positions for later calibration work."""

from __future__ import annotations

import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping

from pwm_output import PwmOutput


_CSV_FIELDS = (
    "recorded_at_utc",
    "position_name",
    "joint_name",
    "channel",
    "legacy_command_deg",
    "pulse_count",
    "pulse_us",
    "duty_cycle",
    "outside_legacy_endpoint",
)


class PositionRecorder:
    """Append exact PWM snapshots to CSV and a structured JSON file."""

    def __init__(self, csv_path: str | Path) -> None:
        self.csv_path = Path(csv_path)
        self.json_path = self.csv_path.with_suffix(".json")
        self._position_number = self._find_next_position_number()

    def _find_next_position_number(self) -> int:
        if not self.json_path.exists():
            return 1
        try:
            data = json.loads(self.json_path.read_text(encoding="utf-8"))
            positions = data.get("positions", {})
            if not isinstance(positions, dict):
                return 1
            numbers = []
            for name in positions:
                if not isinstance(name, str) or not name.startswith("position_"):
                    continue
                try:
                    numbers.append(int(name.removeprefix("position_")))
                except ValueError:
                    continue
            return max(numbers, default=0) + 1
        except (OSError, ValueError, TypeError):
            return 1

    def next_position_name(self) -> str:
        return f"position_{self._position_number:03d}"

    def record(
        self,
        outputs: Mapping[str, PwmOutput],
        *,
        count_min: int,
        count_max: int,
    ) -> str:
        if not outputs:
            raise ValueError("Cannot record a position without PWM outputs")

        position_name = self.next_position_name()
        timestamp = datetime.now(timezone.utc).isoformat()
        ordered = sorted(outputs.values(), key=lambda output: output.channel)

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        file_exists = self.csv_path.exists()
        with self.csv_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=_CSV_FIELDS)
            if not file_exists:
                writer.writeheader()
            for output in ordered:
                writer.writerow(
                    {
                        "recorded_at_utc": timestamp,
                        "position_name": position_name,
                        "joint_name": output.joint_name,
                        "channel": output.channel,
                        "legacy_command_deg": output.command_angle_deg,
                        "pulse_count": output.pulse_count,
                        "pulse_us": f"{output.pulse_us:.9f}",
                        "duty_cycle": output.duty_cycle,
                        "outside_legacy_endpoint": not (
                            count_min <= output.pulse_count <= count_max
                        ),
                    }
                )

        self._update_json(position_name, timestamp, ordered, count_min, count_max)
        self._position_number += 1
        return position_name

    def _update_json(
        self,
        position_name: str,
        timestamp: str,
        outputs: list[PwmOutput],
        count_min: int,
        count_max: int,
    ) -> None:
        data: dict[str, object]
        if self.json_path.exists():
            try:
                loaded = json.loads(self.json_path.read_text(encoding="utf-8"))
                data = loaded if isinstance(loaded, dict) else {}
            except (OSError, ValueError, TypeError):
                data = {}
        else:
            data = {}

        data.setdefault("schema_version", "1.0")
        positions = data.setdefault("positions", {})
        if not isinstance(positions, dict):
            positions = {}
            data["positions"] = positions

        positions[position_name] = {
            "recorded_at_utc": timestamp,
            "joints": {
                output.joint_name: {
                    **output.as_dict(),
                    "legacy_endpoint_min_count": count_min,
                    "legacy_endpoint_max_count": count_max,
                    "outside_legacy_endpoint": not (
                        count_min <= output.pulse_count <= count_max
                    ),
                }
                for output in outputs
            },
        }
        self.json_path.write_text(
            json.dumps(data, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
