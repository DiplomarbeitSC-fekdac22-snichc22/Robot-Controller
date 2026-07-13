"""PIA robot controller translated from FINAL_PCA 3.ino for Raspberry Pi."""

from enum import Enum, auto
import logging
import time

import config
from hardware import Controls, Display, ServoBoard
from ps2_controller import PSB_L1, PSB_R1, ControllerState, Ps2Controller


class Mode(Enum):
    IDLE = auto()
    GAME = auto()
    TEST = auto()


class RobotController:
    def __init__(self) -> None:
        self.controls = Controls()
        self.display = Display()
        self.servos = ServoBoard()
        self.gamepad = Ps2Controller()

        self.mode = Mode.IDLE
        self.control_enabled = False
        self.timer_enabled = False
        self.game_started_at = 0.0
        self.end_message_started_at = 0.0
        self.showing_end_message = False
        self.test_press_started_at: float | None = None

        self.controller_connected = False
        self.last_good_controller_read = 0.0
        self.next_controller_retry = 0.0
        self.last_move = 0.0
        self.last_display_update = 0.0

        self.servo_positions = {
            name: servo.initial_angle for name, servo in config.SERVOS.items()
        }
        self.set_mode(Mode.IDLE)

    def set_mode(self, new_mode: Mode) -> None:
        self.mode = new_mode

        if new_mode is Mode.IDLE:
            self.control_enabled = False
            self.timer_enabled = False
            self.controls.set_game_light(False)
            self.display.show_screen("Spiel starten")

        elif new_mode is Mode.GAME:
            self.control_enabled = True
            self.timer_enabled = True
            self.game_started_at = time.monotonic()
            self.showing_end_message = False
            self.controls.set_game_light(True)
            self.display.show_screen("Spiel gestartet", show_timer=True, time_left=60)

        elif new_mode is Mode.TEST:
            self.control_enabled = True
            self.timer_enabled = False
            self.showing_end_message = False
            self.display.show_screen("TESTMODUS")

    def _handle_buttons(self, now: float) -> bool:
        if self.mode is Mode.IDLE and self.controls.start_button.is_pressed:
            self.set_mode(Mode.GAME)

        if self.mode is not Mode.IDLE and self.controls.stop_button.is_pressed:
            self.set_mode(Mode.IDLE)
            time.sleep(0.300)
            return True

        if self.mode is Mode.IDLE:
            if self.controls.test_button.is_pressed:
                if self.test_press_started_at is None:
                    self.test_press_started_at = now
                elif now - self.test_press_started_at > config.TEST_HOLD_SECONDS:
                    self.set_mode(Mode.TEST)
                    self.test_press_started_at = None
            else:
                self.test_press_started_at = None

        return False

    def _handle_game_timer(self, now: float) -> None:
        if (
            self.mode is Mode.GAME
            and now - self.game_started_at >= config.GAME_DURATION_SECONDS
        ):
            self.set_mode(Mode.IDLE)
            self.display.show_screen("Spiel vorbei")
            self.showing_end_message = True
            self.end_message_started_at = now

        if (
            self.showing_end_message
            and now - self.end_message_started_at > config.END_MESSAGE_SECONDS
        ):
            self.set_mode(Mode.IDLE)
            self.showing_end_message = False

    def _read_controller(self, now: float) -> ControllerState | None:
        if not self.controller_connected:
            if now < self.next_controller_retry:
                return None

            self.next_controller_retry = now + config.CONTROLLER_RETRY_SECONDS
            if self.gamepad.begin():
                self.controller_connected = True
                self.last_good_controller_read = now
                logging.info("Controller connected")
            return None

        state = self.gamepad.read()
        if state is None:
            if now - self.last_good_controller_read > config.CONTROLLER_TIMEOUT_SECONDS:
                self.controller_connected = False
                self.next_controller_retry = now + config.CONTROLLER_RETRY_SECONDS
                logging.warning("Controller lost")
            return None

        self.last_good_controller_read = now
        return state

    @staticmethod
    def _axis_change(value: int, low_change: int, high_change: int) -> int:
        if value < config.DEAD_ZONE_MIN:
            return low_change
        if value > config.DEAD_ZONE_MAX:
            return high_change
        return 0

    def _move_servos(self, state: ControllerState, now: float) -> None:
        if not self.control_enabled:
            return

        if now - self.last_move > config.MOVE_INTERVAL_SECONDS:
            self.last_move = now
            self.servo_positions["base"] += self._axis_change(
                state.left_x, config.SERVOS["base"].speed, -config.SERVOS["base"].speed
            )
            self.servo_positions["shoulder"] += self._axis_change(
                state.left_y,
                -config.SERVOS["shoulder"].speed,
                config.SERVOS["shoulder"].speed,
            )
            self.servo_positions["elbow"] += self._axis_change(
                state.right_y,
                config.SERVOS["elbow"].speed,
                -config.SERVOS["elbow"].speed,
            )
            self.servo_positions["wrist"] += self._axis_change(
                state.right_x,
                config.SERVOS["wrist"].speed,
                -config.SERVOS["wrist"].speed,
            )

            if state.pressed(PSB_R1):
                self.servos.set_angle(config.GRIPPER_CHANNEL, config.GRIPPER_R1_ANGLE)
            if state.pressed(PSB_L1):
                self.servos.set_angle(config.GRIPPER_CHANNEL, config.GRIPPER_L1_ANGLE)

        for name, servo in config.SERVOS.items():
            self.servo_positions[name] = max(
                servo.minimum_angle,
                min(servo.maximum_angle, self.servo_positions[name]),
            )
            self.servos.set_angle(servo.channel, self.servo_positions[name])

    def _update_display_timer(self, now: float) -> None:
        if not self.timer_enabled:
            return
        if now - self.last_display_update <= config.DISPLAY_UPDATE_SECONDS:
            return

        self.last_display_update = now
        elapsed = now - self.game_started_at
        time_left = max(0, int(config.GAME_DURATION_SECONDS - elapsed))
        self.display.update_timer(time_left)

    def run(self) -> None:
        logging.info("PIA robot controller started")
        try:
            while True:
                now = time.monotonic()
                if self._handle_buttons(now):
                    continue
                self._handle_game_timer(now)

                state = self._read_controller(now)
                if state is not None:
                    self._move_servos(state, now)

                self._update_display_timer(now)
                time.sleep(config.LOOP_DELAY_SECONDS)
        except KeyboardInterrupt:
            logging.info("Stopping")
        finally:
            self.close()

    def close(self) -> None:
        self.controls.close()
        self.display.close()
        self.servos.close()
        self.gamepad.close()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
    )
    RobotController().run()


if __name__ == "__main__":
    main()

