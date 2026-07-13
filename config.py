"""Configuration for the PIA robot controller.

GPIO numbers use Raspberry Pi BCM numbering. Adjust them here if the robot is
wired differently.
"""

from dataclasses import dataclass


# GPIO (BCM numbering)
START_BUTTON_PIN = 17
STOP_BUTTON_PIN = 27
TEST_BUTTON_PIN = 22
LED_RELAY_PIN = 23

# I2C devices
LCD_ADDRESS = 0x27
LCD_COLUMNS = 20
LCD_ROWS = 4
PCA9685_ADDRESS = 0x40
PCA9685_FREQUENCY_HZ = 50

# PS2 controller on /dev/spidev0.0 (CE0 is the ATT/select line)
PS2_SPI_BUS = 0
PS2_SPI_DEVICE = 0
PS2_SPI_SPEED_HZ = 250_000

# Timing
GAME_DURATION_SECONDS = 60.0
END_MESSAGE_SECONDS = 2.0
TEST_HOLD_SECONDS = 2.0
MOVE_INTERVAL_SECONDS = 0.030
DISPLAY_UPDATE_SECONDS = 0.500
CONTROLLER_TIMEOUT_SECONDS = 0.500
CONTROLLER_RETRY_SECONDS = 0.500
LOOP_DELAY_SECONDS = 0.015

# Analog-stick dead zone and movement speeds
DEAD_ZONE_MIN = 110
DEAD_ZONE_MAX = 145

# The PCA9685 pulse-count range used by the Arduino sketch.
SERVO_PULSE_MIN = 150
SERVO_PULSE_MAX = 600


@dataclass(frozen=True)
class ServoConfig:
    channel: int
    initial_angle: int
    minimum_angle: int
    maximum_angle: int
    speed: int


SERVOS = {
    "base": ServoConfig(0, 110, 0, 230, 1),
    "shoulder": ServoConfig(2, 67, 50, 95, 1),
    "elbow": ServoConfig(4, 65, 65, 150, 1),
    "wrist": ServoConfig(6, 45, 0, 100, 2),
}

GRIPPER_CHANNEL = 8
GRIPPER_R1_ANGLE = 0
GRIPPER_L1_ANGLE = 90

