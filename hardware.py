"""Raspberry Pi hardware adapters for the LCD, buttons, relay and PCA9685."""

import board
import busio
from adafruit_pca9685 import PCA9685
from gpiozero import Button, DigitalOutputDevice
from RPLCD.i2c import CharLCD

import config


def angle_to_pulse(angle: int) -> int:
    """Match Arduino map(angle, 0, 180, 150, 600), including >180 values."""
    return config.SERVO_PULSE_MIN + (
        angle * (config.SERVO_PULSE_MAX - config.SERVO_PULSE_MIN) // 180
    )


class ServoBoard:
    def __init__(self) -> None:
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._pca = PCA9685(self._i2c, address=config.PCA9685_ADDRESS)
        self._pca.frequency = config.PCA9685_FREQUENCY_HZ

    def set_angle(self, channel: int, angle: int) -> None:
        pulse_count = angle_to_pulse(angle)
        if not 0 <= pulse_count <= 4095:
            raise ValueError(f"Servo pulse count is out of range: {pulse_count}")

        # PCA9685 has 12-bit counts; the CircuitPython driver accepts 16-bit duty.
        self._pca.channels[channel].duty_cycle = pulse_count << 4

    def close(self) -> None:
        self._pca.deinit()
        self._i2c.deinit()


class Display:
    def __init__(self) -> None:
        self._lcd = CharLCD(
            i2c_expander="PCF8574",
            address=config.LCD_ADDRESS,
            port=1,
            cols=config.LCD_COLUMNS,
            rows=config.LCD_ROWS,
            charmap="A00",
            auto_linebreaks=False,
        )
        self._lcd.backlight_enabled = True

    def _write_at(self, row: int, column: int, text: str) -> None:
        self._lcd.cursor_pos = (row, column)
        self._lcd.write_string(text)

    def show_screen(
        self,
        bottom_text: str,
        show_timer: bool = False,
        time_left: int = 0,
    ) -> None:
        self._lcd.clear()
        self._write_at(0, 3, "PIA AUTOMATION")
        self._write_at(1, 6, "ROBI V01")

        if show_timer:
            self.update_timer(time_left)

        text = bottom_text[: config.LCD_COLUMNS]
        column = max(0, (config.LCD_COLUMNS - len(text)) // 2)
        self._write_at(3, column, text)

    def update_timer(self, time_left: int) -> None:
        # Fixed-width output clears digits left over from the previous value.
        self._write_at(2, 6, f"Zeit: {time_left:>3}s ")

    def close(self) -> None:
        self._lcd.clear()
        self._lcd.close(clear=False)


class Controls:
    def __init__(self) -> None:
        self.start_button = Button(
            config.START_BUTTON_PIN, pull_up=True, bounce_time=0.05
        )
        self.stop_button = Button(
            config.STOP_BUTTON_PIN, pull_up=True, bounce_time=0.05
        )
        self.test_button = Button(
            config.TEST_BUTTON_PIN, pull_up=True, bounce_time=0.05
        )
        # HIGH in idle and LOW while the game is active, like the Arduino code.
        self.led_relay = DigitalOutputDevice(
            config.LED_RELAY_PIN, active_high=True, initial_value=True
        )

    def set_game_light(self, enabled: bool) -> None:
        if enabled:
            self.led_relay.off()  # LOW: active-low relay enabled
        else:
            self.led_relay.on()  # HIGH: relay disabled

    def close(self) -> None:
        self.set_game_light(False)
        self.start_button.close()
        self.stop_button.close()
        self.test_button.close()
        self.led_relay.close()

