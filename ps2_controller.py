"""Minimal PlayStation 2 controller driver using Raspberry Pi SPI0/CE0."""

from dataclasses import dataclass
import time
from typing import Optional, Sequence

import spidev

import config


# Button bits after converting the controller's active-low word to active-high.
PSB_L1 = 1 << 10
PSB_R1 = 1 << 11


@dataclass(frozen=True)
class ControllerState:
    left_x: int
    left_y: int
    right_x: int
    right_y: int
    buttons: int

    def pressed(self, button_mask: int) -> bool:
        return bool(self.buttons & button_mask)


def _reverse_bits(value: int) -> int:
    value = ((value & 0xF0) >> 4) | ((value & 0x0F) << 4)
    value = ((value & 0xCC) >> 2) | ((value & 0x33) << 2)
    return ((value & 0xAA) >> 1) | ((value & 0x55) << 1)


class Ps2Controller:
    """Poll a wired PS2 controller or receiver using the PS2 serial protocol.

    PS2 transfers are least-significant-bit first. Raspberry Pi's SPI controller
    normally transfers most-significant-bit first, so each byte is reversed
    before and after the transfer.
    """

    _POLL = (0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)

    def __init__(self) -> None:
        self._spi = spidev.SpiDev()
        self._spi.open(config.PS2_SPI_BUS, config.PS2_SPI_DEVICE)
        self._spi.max_speed_hz = config.PS2_SPI_SPEED_HZ
        self._spi.mode = 0b11
        self._spi.bits_per_word = 8

    def _exchange(self, command: Sequence[int]) -> list[int]:
        transmitted = [_reverse_bits(byte) for byte in command]
        received = self._spi.xfer2(transmitted)
        return [_reverse_bits(byte) for byte in received]

    @staticmethod
    def _valid_response(response: Sequence[int]) -> bool:
        return len(response) >= 5 and response[2] == 0x5A

    @staticmethod
    def _is_analog_response(response: Sequence[int]) -> bool:
        return len(response) >= 9 and (response[1] & 0xF0) == 0x70

    def begin(self) -> bool:
        """Find the controller and switch it into locked analog mode."""
        response = self._exchange(self._POLL)
        if self._valid_response(response) and self._is_analog_response(response):
            return True

        # Enter configuration, select analog mode, then leave configuration.
        commands = (
            (0x01, 0x43, 0x00, 0x01, 0x00),
            (0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00),
            (0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A),
        )
        for command in commands:
            self._exchange(command)
            time.sleep(0.010)

        response = self._exchange(self._POLL)
        return self._valid_response(response) and self._is_analog_response(response)

    def read(self) -> Optional[ControllerState]:
        response = self._exchange(self._POLL)
        if not self._valid_response(response) or not self._is_analog_response(response):
            return None

        raw_buttons = response[3] | (response[4] << 8)
        buttons = (~raw_buttons) & 0xFFFF
        return ControllerState(
            right_x=response[5],
            right_y=response[6],
            left_x=response[7],
            left_y=response[8],
            buttons=buttons,
        )

    def close(self) -> None:
        self._spi.close()

