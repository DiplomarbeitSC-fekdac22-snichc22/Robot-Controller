from __future__ import annotations

import math


LCD_I2C_BUS = 20
LCD_I2C_ADDRESS = 0x27
LCD_COLUMNS = 20
LCD_ROWS = 4
START_BUTTON_GPIO = 5
STOP_BUTTON_GPIO = 6
# Physical pin 24; the existing relay input is active-low.
LED_RELAY_GPIO = 8


class MovementPermission:
    def __init__(self, duration_seconds: float = 90.0, notice_seconds: float = 10.0):
        if not all(math.isfinite(value) and value > 0 for value in (duration_seconds, notice_seconds)):
            raise ValueError("Permission and notice durations must be finite and positive")
        self.duration_seconds = duration_seconds
        self.notice_seconds = notice_seconds
        self.state = "standby"
        self.ends_at = 0.0
        self.notice_until = 0.0
        self.reason = None

    @property
    def allowed(self) -> bool:
        return self.state in ("timed", "setup")

    def start(self, now: float, setup: bool = False) -> bool:
        self.update(now)
        if self.state in ("cancelled", "expired"):
            return False
        self.state = "setup" if setup else "timed"
        self.ends_at = 0.0 if setup else now + self.duration_seconds
        self.reason = None
        return True

    def cancel(self, now: float, reason: str = "cancelled") -> None:
        if self.allowed or self.reason != reason:
            self.state = "cancelled"
            self.notice_until = now + self.notice_seconds
            self.reason = reason
        self.ends_at = 0.0

    def update(self, now: float) -> bool:
        expired = self.state == "timed" and now >= self.ends_at
        if expired:
            self.state = "expired"
            self.ends_at = 0.0
            self.notice_until = now + self.notice_seconds
            self.reason = "expired"
        elif self.state in ("expired", "cancelled") and now >= self.notice_until:
            self.state = "standby"
        return expired


class GameDisplay:
    def __init__(self, duration_seconds: float = 90.0, setup_button_gpio: int | None = None,
                 emergency_stop_gpio: int | None = None,
                 emergency_stop_healthy_pressed: bool | None = None,
                 notice_seconds: float = 10.0, *, devices=None):
        self.permission = MovementPermission(duration_seconds, notice_seconds)
        self.duration_seconds = duration_seconds
        self.operation_mode = "manual"
        self._screen = None
        self._closed = False
        self._devices = []
        self.lcd = None
        self.game_light = None
        self.setup_button = None
        self.emergency_stop_button = None
        self.emergency_stop_healthy_pressed = emergency_stop_healthy_pressed
        self._armed = False
        pins = [START_BUTTON_GPIO, STOP_BUTTON_GPIO, LED_RELAY_GPIO]
        pins += [pin for pin in (setup_button_gpio, emergency_stop_gpio) if pin is not None]
        if len(pins) != len(set(pins)) or any(type(pin) is not int or not 0 <= pin <= 27 for pin in pins):
            raise ValueError("Button and relay GPIO pins must be distinct BCM numbers from 0 to 27")
        if (emergency_stop_healthy_pressed is not None and emergency_stop_healthy_pressed is not True
                or devices is None and (emergency_stop_gpio is None) != (emergency_stop_healthy_pressed is None)):
            raise ValueError("Emergency feedback requires a verified NC contact to GND and healthy_pressed=true")
        try:
            if devices is None:
                from gpiozero import Button, DigitalOutputDevice
                from RPLCD.i2c import CharLCD

                self.lcd = CharLCD(i2c_expander="PCF8574", address=LCD_I2C_ADDRESS,
                                   port=LCD_I2C_BUS, cols=LCD_COLUMNS, rows=LCD_ROWS,
                                   charmap="A00", auto_linebreaks=False)
                self.lcd.backlight_enabled = True
                self.start_button = Button(START_BUTTON_GPIO, pull_up=True, bounce_time=0.05)
                self._devices.append(self.start_button)
                self.stop_button = Button(STOP_BUTTON_GPIO, pull_up=True, bounce_time=0.05)
                self._devices.append(self.stop_button)
                if setup_button_gpio is not None:
                    self.setup_button = Button(setup_button_gpio, pull_up=True, bounce_time=0.05)
                    self._devices.append(self.setup_button)
                if emergency_stop_gpio is not None:
                    # Supported feedback: a verified NC contact to GND; an open loop inhibits motion.
                    self.emergency_stop_button = Button(emergency_stop_gpio, pull_up=True)
                    self._devices.append(self.emergency_stop_button)
                self.game_light = DigitalOutputDevice(LED_RELAY_GPIO, active_high=False, initial_value=False)
                self._devices.append(self.game_light)
            else:
                self.lcd = devices.lcd
                self.start_button = devices.start_button
                self.stop_button = devices.stop_button
                self.setup_button = getattr(devices, "setup_button", None)
                self.emergency_stop_button = getattr(devices, "emergency_stop_button", None)
                self.game_light = devices.game_light
                self._devices = [self.start_button, self.stop_button, self.game_light]
                self._devices += [button for button in (self.setup_button, self.emergency_stop_button) if button is not None]
            if self.emergency_stop_button is not None and emergency_stop_healthy_pressed is None:
                raise ValueError("Emergency-stop contact polarity has not been configured")
            self.previous_start = self.start_button.is_pressed
            self.previous_stop = self.stop_button.is_pressed
            self.previous_setup = self._setup_pressed()
            self.previous_emergency = self.emergency_stop_active
            self.show_ready()
        except Exception:
            self.close()
            raise

    @property
    def movement_allowed(self) -> bool:
        return self.permission.allowed and not self.emergency_stop_active and not self.stop_button.is_pressed

    @property
    def game_active(self) -> bool:
        return self.movement_allowed

    @property
    def game_ends_at(self) -> float:
        return self.permission.ends_at

    @property
    def emergency_stop_available(self) -> bool:
        return self.emergency_stop_button is not None

    @property
    def emergency_stop_active(self) -> bool:
        return self.emergency_stop_button is not None and (
            self.emergency_stop_button.is_pressed != self.emergency_stop_healthy_pressed
        )

    def _setup_pressed(self) -> bool:
        return self.setup_button is not None and self.setup_button.is_pressed

    def _set_game_light(self, enabled: bool) -> None:
        if self.game_light is not None:
            self.game_light.on() if enabled else self.game_light.off()

    @staticmethod
    def _fit(text: str, centered: bool = True) -> str:
        text = text[:LCD_COLUMNS]
        return text.center(LCD_COLUMNS) if centered else text.ljust(LCD_COLUMNS)

    def _render(self, now: float, armed: bool) -> None:
        self._armed = armed
        self._set_game_light(self.movement_allowed and armed)
        state = self.permission.state
        if self.emergency_stop_active:
            lines = ("PIA-AUTOMATION", "NOT-HALT AKTIV", "FREIGABE GESPERRT", "RESET + NEUER START")
        elif state == "timed":
            seconds = max(0, math.ceil(self.game_ends_at - now))
            lines = ("PIA-AUTOMATION", f"Time Limit {seconds} Sec.", "", "ARM: ENABLED" if armed else "ARM: DISABLED")
        elif state == "setup":
            lines = ("PIA-AUTOMATION", "Einstellbetrieb", "BIS ROTEM ABBRUCH", "ARM: ENABLED" if armed else "ARM: DISABLED")
        elif state == "expired":
            lines = ("PIA-AUTOMATION", "Game over", "FREIGABE BEENDET", "PWM DISABLED")
        elif state == "cancelled":
            lines = ("PIA-AUTOMATION", "Abbruch der Freigabe", "PWM DISABLED", "NEUER START NOETIG")
        else:
            lines = ("PIA-AUTOMATION", "Are you Ready?", "Let's start", "PRESS GREEN START")
        if not self.emergency_stop_active:
            if self.operation_mode == "automatic":
                mode_line = "AUTO: NICHT BEREIT"
            elif state == "standby":
                mode_line = "MANUAL: GRUEN START"
            else:
                mode_line = "MANUAL: ARM EIN" if armed and self.movement_allowed else "MANUAL: PWM AUS"
            lines = (*lines[:3], mode_line)
        if lines != self._screen:
            for row, line in enumerate(lines):
                self.lcd.cursor_pos = (row, 0)
                self.lcd.write_string(self._fit(line))
            self._screen = lines

    def show_ready(self) -> None:
        self.permission.state = "standby"
        self.permission.ends_at = 0.0
        self.permission.reason = None
        self._render(0.0, False)

    def start_game(self, now: float, armed: bool) -> bool:
        if self.emergency_stop_active or self.stop_button.is_pressed:
            return False
        started = self.permission.start(now)
        self._render(now, armed)
        return started

    def cancel(self, now: float, reason: str = "cancelled") -> None:
        self.permission.cancel(now, reason)
        self.previous_start = self.start_button.is_pressed
        self.previous_setup = self._setup_pressed()
        self._render(now, False)

    def show_game_over_complete(self, home_reached: bool) -> None:
        # Kept for older callers; completing a return never grants movement.
        self._render(self.permission.notice_until - self.permission.notice_seconds, False)

    def refresh(self, now: float, armed: bool) -> None:
        if not self._closed:
            self._render(now, armed)

    def update(self, now: float, armed: bool) -> str | None:
        start_pressed = self.start_button.is_pressed
        stop_pressed = self.stop_button.is_pressed
        setup_pressed = self._setup_pressed()
        emergency_active = self.emergency_stop_active
        expired = self.permission.update(now)
        event = "game_over" if expired else None
        if emergency_active:
            if not self.previous_emergency or self.permission.allowed:
                event = "emergency_stop"
            self.permission.cancel(now, "emergency_stop")
        elif stop_pressed:
            if not self.previous_stop or self.permission.allowed:
                event = "game_stopped"
            self.permission.cancel(now)
        elif setup_pressed and not self.previous_setup:
            if self.permission.start(now, setup=True):
                event = "setup_started"
        elif start_pressed and not self.previous_start and not self.permission.allowed:
            if self.permission.start(now):
                event = "game_started"
        self.previous_start = start_pressed
        self.previous_stop = stop_pressed
        self.previous_setup = setup_pressed
        self.previous_emergency = emergency_active
        self._render(now, armed)
        return event

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.permission.state = "standby"
        self.permission.ends_at = 0.0
        try:
            self._set_game_light(False)
        finally:
            for device in self._devices:
                try:
                    device.close()
                except Exception:
                    pass
            if self.lcd is not None:
                try:
                    self.lcd.clear()
                finally:
                    self.lcd.close(clear=False)
