import unittest

import config
from keyboard_control import KeyboardController


class KeyboardControllerTests(unittest.TestCase):
    def test_base_moves_by_configured_step(self) -> None:
        controller = KeyboardController()
        initial_angle = controller.positions["base"]

        controller.handle_key("d")

        self.assertEqual(
            controller.positions["base"],
            initial_angle + config.KEYBOARD_STEP_DEGREES,
        )


if __name__ == "__main__":
    unittest.main()
