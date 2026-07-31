from __future__ import annotations

import math

from gpiozero import Button, DigitalOutputDevice
from RPLCD.i2c import CharLCD


LCD_I2C_BUS = 20
LCD_I2C_ADDRESS = 0x27
LCD_COLUMNS = 20
LCD_ROWS = 4

START_BUTTON_GPIO = 5
STOP_BUTTON_GPIO = 6

# Physical pin 24 = BCM GPIO8.
# The relay input is active-low:
#   GPIO HIGH -> relay/LED off
#   GPIO LOW  -> relay/LED on
LED_RELAY_GPIO = 8


class GameDisplay:
    def __init__(
        self,
        duration_seconds: float = 30.0,
    ) -> None:
        self.duration_seconds = duration_seconds

        self.lcd = CharLCD(
            i2c_expander="PCF8574",
            address=LCD_I2C_ADDRESS,
            port=LCD_I2C_BUS,
            cols=LCD_COLUMNS,
            rows=LCD_ROWS,
            charmap="A00",
            auto_linebreaks=False,
        )

        self.lcd.backlight_enabled = True

        self.start_button = Button(
            START_BUTTON_GPIO,
            pull_up=True,
            bounce_time=0.05,
        )

        self.stop_button = Button(
            STOP_BUTTON_GPIO,
            pull_up=True,
            bounce_time=0.05,
        )

        self.game_light = DigitalOutputDevice(
            LED_RELAY_GPIO,
            active_high=False,
            initial_value=False,
        )

        self.game_active = False
        self.game_ends_at = 0.0
        self.last_seconds: int | None = None
        self.last_armed: bool | None = None

        self.previous_start = (
            self.start_button.is_pressed
        )
        self.previous_stop = (
            self.stop_button.is_pressed
        )

        self.show_ready()

    def _set_game_light(
        self,
        enabled: bool,
    ) -> None:
        if enabled:
            self.game_light.on()
        else:
            self.game_light.off()

    @staticmethod
    def _fit(
        text: str,
        centered: bool = False,
    ) -> str:
        text = text[:LCD_COLUMNS]

        if centered:
            return text.center(LCD_COLUMNS)

        return text.ljust(LCD_COLUMNS)

    def _line(
        self,
        row: int,
        text: str,
        centered: bool = False,
    ) -> None:
        self.lcd.cursor_pos = (row, 0)

        self.lcd.write_string(
            self._fit(
                text,
                centered,
            )
        )

    def show_ready(self) -> None:
        self._set_game_light(False)

        self.game_active = False
        self.last_seconds = None
        self.last_armed = None

        self._line(
            0,
            "PIA AUTOMATION",
            centered=True,
        )
        self._line(
            1,
            "ROBI V01",
            centered=True,
        )
        self._line(
            2,
            "READY",
            centered=True,
        )
        self._line(
            3,
            "PRESS GREEN START",
            centered=True,
        )

    def start_game(
        self,
        now: float,
        armed: bool,
    ) -> None:
        self._set_game_light(True)

        self.game_active = True
        self.game_ends_at = (
            now + self.duration_seconds
        )

        self.last_seconds = None
        self.last_armed = None

        self._line(
            0,
            "PIA AUTOMATION",
            centered=True,
        )
        self._line(
            1,
            "GAME START",
            centered=True,
        )

        self._update_game_screen(
            now,
            armed,
        )

    def show_game_over(self) -> None:
        self._set_game_light(False)

        self.game_active = False
        self.last_seconds = None
        self.last_armed = None

        self._line(
            0,
            "PIA AUTOMATION",
            centered=True,
        )
        self._line(
            1,
            "GAME OVER",
            centered=True,
        )
        self._line(
            2,
            "OPENING GRIPPER",
            centered=True,
        )
        self._line(
            3,
            "RETURNING HOME",
            centered=True,
        )

    def show_game_over_complete(
        self,
        home_reached: bool,
    ) -> None:
        self._set_game_light(False)

        self.game_active = False
        self.last_seconds = None
        self.last_armed = None

        self._line(
            0,
            "PIA AUTOMATION",
            centered=True,
        )
        self._line(
            1,
            "GAME OVER",
            centered=True,
        )

        if home_reached:
            self._line(
                2,
                "HOME - GRIPPER OPEN",
                centered=True,
            )
        else:
            self._line(
                2,
                "PWM WAS DISABLED",
                centered=True,
            )

        self._line(
            3,
            "PRESS GREEN START",
            centered=True,
        )

    def show_stopped(self) -> None:
        self._set_game_light(False)

        self.game_active = False
        self.last_seconds = None
        self.last_armed = None

        self._line(
            0,
            "PIA AUTOMATION",
            centered=True,
        )
        self._line(
            1,
            "GAME STOPPED",
            centered=True,
        )
        self._line(
            2,
            "PWM DISABLED",
            centered=True,
        )
        self._line(
            3,
            "PRESS GREEN START",
            centered=True,
        )

    def _update_game_screen(
        self,
        now: float,
        armed: bool,
    ) -> None:
        seconds = max(
            0,
            math.ceil(
                self.game_ends_at - now
            ),
        )

        if seconds != self.last_seconds:
            self._line(
                2,
                f"TIME LEFT: {seconds:02d} s",
                centered=True,
            )

            self.last_seconds = seconds

        if armed != self.last_armed:
            arm_text = (
                "ARM: ENABLED"
                if armed
                else "ARM: DISABLED"
            )

            self._line(
                3,
                arm_text,
                centered=True,
            )

            self.last_armed = armed

    def update(
        self,
        now: float,
        armed: bool,
    ) -> str | None:
        start_pressed = (
            self.start_button.is_pressed
        )
        stop_pressed = (
            self.stop_button.is_pressed
        )

        event: str | None = None

        if (
            start_pressed
            and not self.previous_start
            and not self.game_active
        ):
            self.start_game(
                now,
                armed,
            )
            event = "game_started"

        if (
            stop_pressed
            and not self.previous_stop
        ):
            self.show_stopped()
            event = "game_stopped"

        self.previous_start = start_pressed
        self.previous_stop = stop_pressed

        if self.game_active:
            if now >= self.game_ends_at:
                self.show_game_over()
                event = "game_over"
            else:
                self._update_game_screen(
                    now,
                    armed,
                )

        return event

    def close(self) -> None:
        try:
            self._set_game_light(False)
            self.game_light.close()
        finally:
            self.start_button.close()
            self.stop_button.close()
            self.lcd.clear()
            self.lcd.close(clear=False)
