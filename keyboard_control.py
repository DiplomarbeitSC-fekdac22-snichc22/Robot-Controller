import os
import sys
import tty

import termios

import config


MOVEMENT_KEYS = {
    "a": ("base", -1),
    "d": ("base", 1),
    "w": ("shoulder", 1),
    "s": ("shoulder", -1),
    "i": ("elbow", 1),
    "k": ("elbow", -1),
    "j": ("wrist", -1),
    "l": ("wrist", 1),
}


class KeyboardController:
    def __init__(self) -> None:
        self.positions = {
            name: servo.initial_angle
            for name, servo in config.SERVOS.items()
        }
        self.gripper_angle = config.GRIPPER_R1_ANGLE

    def handle_key(self, key: str) -> bool:
        key = key.lower()

        if key == "q":
            return False

        if key in MOVEMENT_KEYS:
            name, direction = MOVEMENT_KEYS[key]
            servo = config.SERVOS[name]

            new_angle = (
                self.positions[name]
                + direction * config.KEYBOARD_STEP_DEGREES
            )

            self.positions[name] = max(
                servo.minimum_angle,
                min(servo.maximum_angle, new_angle),
            )

        elif key == "o":
            self.gripper_angle = config.GRIPPER_R1_ANGLE

        elif key == "c":
            self.gripper_angle = config.GRIPPER_L1_ANGLE

        return True

    def display(self) -> None:
        # Clear screen and move cursor to the top-left corner.
        print("\033[2J\033[H", end="")

        print("Robot keyboard control")
        print("======================")
        print("a/d  base -/+")
        print("w/s  shoulder +/-")
        print("i/k  elbow +/-")
        print("j/l  wrist -/+")
        print("o/c  gripper open/close")
        print("q    quit")
        print()
        print(f"Base:     {self.positions['base']:>3} deg")
        print(f"Shoulder: {self.positions['shoulder']:>3} deg")
        print(f"Elbow:    {self.positions['elbow']:>3} deg")
        print(f"Wrist:    {self.positions['wrist']:>3} deg")
        print(f"Gripper:  {self.gripper_angle:>3} deg")
        print()
        print("Press or hold a movement key.", flush=True)

    def run(self) -> None:
        file_descriptor = sys.stdin.fileno()
        old_settings = termios.tcgetattr(file_descriptor)

        try:
            tty.setcbreak(file_descriptor)

            self.display()

            while True:
                key = os.read(file_descriptor, 1).decode(
                    "utf-8",
                    errors="ignore",
                )

                if not key:
                    continue

                if not self.handle_key(key):
                    break

                self.display()

        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(
                file_descriptor,
                termios.TCSADRAIN,
                old_settings,
            )

            print("\033[2J\033[H", end="")
            print("Keyboard control stopped.")


def main() -> None:
    KeyboardController().run()


if __name__ == "__main__":
    main()
