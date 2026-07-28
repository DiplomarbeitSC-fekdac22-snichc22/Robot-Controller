"""Raspberry Pi hardware adapters for the LCD, buttons, relay and PCA9685."""

from __future__ import annotations

import board
import busio
from adafruit_pca9685 import PCA9685
from gpiozero import Button, DigitalOutputDevice
from RPLCD.i2c import CharLCD

import config
from pwm_output import PwmOutput, build_pwm_output


class BlinkaBoardError(RuntimeError):
    """Raised when the wrong Python ``board`` package shadows Adafruit Blinka."""


def _create_i2c_bus():
    """Create the Raspberry Pi I2C bus and diagnose a shadowed Blinka module."""
    missing = [name for name in ("SCL", "SDA") if not hasattr(board, name)]
    if missing:
        module_path = getattr(board, "__file__", "unknown")
        raise BlinkaBoardError(
            "The imported Python module named 'board' is not the Adafruit Blinka "
            "Raspberry Pi board module. Missing attributes: "
            f"{', '.join(missing)}. Imported from: {module_path}.\n"
            "Activate the Robot-Controller virtual environment and run:\n"
            "  python -m pip uninstall -y board adafruit-pca9685\n"
            "  python -m pip install --upgrade Adafruit-Blinka "
            "adafruit-circuitpython-pca9685\n"
            "Then verify with:\n"
            "  python -c \"import board; print(board.__file__); "
            "print(board.SCL, board.SDA)\""
        )
    return busio.I2C(board.SCL, board.SDA)


def angle_to_pulse(angle: int) -> int:
    """Backward-compatible name returning the legacy PCA9685 count value."""
    return build_pwm_output(
        joint_name="legacy",
        channel=0,
        command_angle_deg=angle,
        count_min=config.SERVO_PULSE_MIN,
        count_max=config.SERVO_PULSE_MAX,
        frequency_hz=config.PCA9685_FREQUENCY_HZ,
        resolution_counts=config.PCA9685_RESOLUTION_COUNTS,
    ).pulse_count


class ServoBoard:
    def __init__(self) -> None:
        self._i2c = _create_i2c_bus()
        try:
            self._pca = PCA9685(self._i2c, address=config.PCA9685_ADDRESS)
            self._pca.frequency = config.PCA9685_FREQUENCY_HZ
        except Exception:
            self._i2c.deinit()
            raise
        self._last_outputs: dict[int, PwmOutput] = {}

    @property
    def last_outputs(self) -> dict[int, PwmOutput]:
        return dict(self._last_outputs)

    def set_angle(
        self,
        channel: int,
        angle: int,
        *,
        joint_name: str | None = None,
    ) -> PwmOutput:
        """Send one legacy angle command and return the exact generated PWM data."""
        output = build_pwm_output(
            joint_name=joint_name or f"channel_{channel}",
            channel=channel,
            command_angle_deg=angle,
            count_min=config.SERVO_PULSE_MIN,
            count_max=config.SERVO_PULSE_MAX,
            frequency_hz=config.PCA9685_FREQUENCY_HZ,
            resolution_counts=config.PCA9685_RESOLUTION_COUNTS,
        )
        self._pca.channels[channel].duty_cycle = output.duty_cycle
        self._last_outputs[channel] = output
        return output

    def set_servo(self, servo: config.ServoConfig, angle: int) -> PwmOutput:
        """Send a command using a loaded servo definition."""
        return self.set_angle(
            servo.channel,
            angle,
            joint_name=servo.joint_name,
        )

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
        self.led_relay = DigitalOutputDevice(
            config.LED_RELAY_PIN, active_high=True, initial_value=True
        )

    def set_game_light(self, enabled: bool) -> None:
        if enabled:
            self.led_relay.off()
        else:
            self.led_relay.on()

    def close(self) -> None:
        self.set_game_light(False)
        self.start_button.close()
        self.stop_button.close()
        self.test_button.close()
        self.led_relay.close()
