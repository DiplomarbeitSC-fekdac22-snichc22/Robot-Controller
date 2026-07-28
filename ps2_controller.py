"""PS2 controller adapter using Joy-IT's official WLGamepad library.

This preserves the interface expected by ``ps2_manual_control.py`` while
using the transfer sequence required by the Joy-IT SBC-WLGamepad receiver.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Final


# The Raspberry Pi 5 header is exposed through gpiochip0 on this installation.
# This must be set before importing RPi.GPIO/rpi-lgpio.
os.environ.setdefault("RPI_LGPIO_CHIP", "0")

import RPi.GPIO as GPIO
from wlgamepad import WLGamepad


# BCM GPIO numbering; these match the robot's verified wiring.
PS2_DATA_GPIO: Final = 25
PS2_COMMAND_GPIO: Final = 13
PS2_ATTENTION_GPIO: Final = 12
PS2_CLOCK_GPIO: Final = 18

PSB_L1: Final = 1 << 10
PSB_R1: Final = 1 << 11

_BUTTON_BITS: Final = {
    "Select": 0,
    "L3": 1,
    "R3": 2,
    "Start": 3,
    "Up": 4,
    "Right": 5,
    "Down": 6,
    "Left": 7,
    "L2": 8,
    "R2": 9,
    "L1": 10,
    "R1": 11,
    "Triangle": 12,
    "Circle": 13,
    "Cross": 14,
    "Square": 15,
}


@dataclass(frozen=True)
class ControllerState:
    left_x: int
    left_y: int
    right_x: int
    right_y: int
    buttons: int

    def pressed(self, button_mask: int) -> bool:
        return bool(self.buttons & button_mask)


class Ps2Controller:
    """Adapt Joy-IT's API to the robot controller's existing interface."""

    def __init__(self) -> None:
        self._closed = False
        self._controller = WLGamepad(
            ATT=PS2_ATTENTION_GPIO,
            CLK=PS2_CLOCK_GPIO,
            CMD=PS2_COMMAND_GPIO,
            DATA=PS2_DATA_GPIO,
        )

    @staticmethod
    def _valid(raw_state: dict) -> bool:
        raw = raw_state.get("raw")
        return (
            isinstance(raw, list)
            and len(raw) >= 3
            and raw[1] != "FF"
            and raw[2] in ("5A", "00")
        )

    @staticmethod
    def _convert(raw_state: dict) -> ControllerState:
        sticks = raw_state["sticks"]
        buttons = 0
        for name in raw_state["buttons"]:
            bit = _BUTTON_BITS.get(name)
            if bit is not None:
                buttons |= 1 << bit

        return ControllerState(
            left_x=int(sticks["LX"]),
            left_y=int(sticks["LY"]),
            right_x=int(sticks["RX"]),
            right_y=int(sticks["RY"]),
            buttons=buttons,
        )

    def begin(self) -> bool:
        if self._closed:
            return False

        self._controller.setup_controller()
        return self._valid(self._controller.read_controller())

    def read(self) -> ControllerState | None:
        if self._closed:
            return None

        raw_state = self._controller.read_controller()
        if not self._valid(raw_state):
            return None
        return self._convert(raw_state)

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True
        GPIO.cleanup(
            [
                PS2_ATTENTION_GPIO,
                PS2_CLOCK_GPIO,
                PS2_COMMAND_GPIO,
                PS2_DATA_GPIO,
            ]
        )
