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
            name: servo.initial_angle for name, servo in config.SERVOS.items()
        }
        self.gripper_angle = config.GRIPPER_R1_ANGLE

    def handle_key(self, key: str) -> bool:
        key = key.strip().lower()

        if key == "q":
            return False
        if key == "o":
            self.gripper_angle = config.GRIPPER_R1_ANGLE
        elif key == "c":
            self.gripper_angle = config.GRIPPER_L1_ANGLE
        elif key in MOVEMENT_KEYS:
            name, direction = MOVEMENT_KEYS[key]
            servo = config.SERVOS[name]
            angle = self.positions[name] + direction * config.KEYBOARD_STEP_DEGREES
            self.positions[name] = max(
                servo.minimum_angle,
                min(servo.maximum_angle, angle),
            )

        return True

    def print_state(self) -> None:
        print("\nCurrent angles:")
        for name, servo in config.SERVOS.items():
            print(f"  {servo.joint_name}: {self.positions[name]} deg")
        print(f"  {config.GRIPPER_SERVO.joint_name}: {self.gripper_angle} deg")


def main() -> None:
    controller = KeyboardController()
    print("a/d base | w/s shoulder | i/k elbow | j/l wrist")
    print("o open | c close | p print | q quit")
    controller.print_state()

    while True:
        if not controller.handle_key(input("> ")):
            break
        controller.print_state()


if __name__ == "__main__":
    main()
