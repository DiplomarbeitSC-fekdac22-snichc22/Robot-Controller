# Current PS2 robot-arm controls

`run_ps2_controller.sh` starts `ps2_manual_control.py`. The active driver uses
Joy-IT's `WLGamepad` transfer library and the existing BCM pins below; the older
SPI wiring documented in `config.py` belongs to the legacy entry point.

## Existing wiring

| Signal | BCM GPIO |
| --- | ---: |
| PS2 DATA | 25 |
| PS2 CMD | 13 |
| PS2 ATT | 12 |
| PS2 CLK | 18 |
| Green start button to GND | 5 |
| Red stop button to GND | 6 |
| Permission LED relay, active-low | 8 |

The PCA9685 address is `0x40`; the existing LCD is at `0x27` on I2C bus 20.
Keep the existing Pi environment and its Joy-IT, GPIO, LCD and PCA9685 libraries.
Do not substitute the legacy GPIO17/27/22/23 button wiring. The white setup button
and optional E-stop feedback need their actual wiring configured under
`[operation]` in `config/ps2_controller.toml`; no GPIO is guessed for them.
Optional E-stop feedback requires a verified supervised normally-closed contact
to GND and `emergency_stop_healthy_pressed = true`; an open/broken contact blocks
permission. The opposite, unverified polarity is rejected.

## Start without hardware

```bash
python ps2_manual_control.py --simulate
```

This exercises controller/permission logic with fake GPIO, display and PWM.
Use the command list printed at startup to grant/cancel permission, change modes,
and inspect simulated output. It cannot move physical servos. Automated tests use
only the Python standard library:

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s tests -v
```

## Pi input diagnosis

With the existing controller environment:

```bash
bash run_ps2_controller.sh --diagnose
```

Move the sticks and press buttons. Neutral stick values should be near 128.
Diagnosis opens the PS2 receiver but never the PCA9685. Stop it with Ctrl+C.

## Manual operation

```bash
bash run_ps2_controller.sh
```

Start with neutral sticks and all controls released. Verify that the physical
arm matches the stored position, then press START to confirm it. This does not
move the arm. Press the cabinet green button for the configured 90-second
permission or the configured white button for setup permission.

| Input | Action |
| --- | --- |
| START | Confirm actual pose against stored pulses, with PWM stopped |
| SELECT | Switch manual/automatic; revoke permission and disable PWM |
| D-pad UP | Toggle PWM within an already granted permission |
| D-pad DOWN | Software stop: revoke permission and disable PWM |
| Left stick X/Y | Base / shoulder, manual mode only |
| Right stick Y/X | Elbow / wrist, manual mode only |
| R1 / L1 | Gripper open / close, manual mode only |
| Green cabinet button | Request timed permission |
| White cabinet button | Request untimed setup permission, if configured |
| Red cabinet button | Revoke permission and stop PWM |
| Ctrl+C | Disable PWM and exit |

A valid connection, confirmed position, neutral controls and manual mode are
required before a new permission can arm the axes. Cancellation, timeout,
disconnection and mode changes invalidate the confirmation. Reconnecting or
releasing a stop cannot restart motion by itself. Hold red to prevent any start.

The permission LED follows actual permitted PWM. The LCD shows timed operation,
setup, cancellation and expiry; end messages remain for about 10 seconds.
There is no automatic gripper opening or home-return after permission expires.
Automatic mode accepts mode/status coordination only and blocks joystick motion;
actual camera pickup is not connected yet.

## Servo and operational settings

`config/ps2_controller.toml` is the active source of measured pulse ranges,
initial angles, channels, directions, dead zones and maximum joint speeds.
Do not use the obsolete 700/1275 microsecond gripper example from earlier notes.
The current configured outputs are:

| Joint | PCA9685 channel |
| --- | ---: |
| Base | 0 |
| Shoulder | 2 |
| Elbow | 4 |
| Wrist | 6 |
| Gripper | 8 |

The motion ramp limits abrupt velocity changes without raising existing axis
speed limits. Gripper targets are approached gradually. Pulse values are stored
commands, not measured joint positions; after a power loss, verify the pose before
confirming it or enabling PWM.

The loopback API defaults to port 8765: `GET /status` and `POST /mode`.
It exposes no servo command or permission-grant route. The backend reaches it
using `ROBOT_CONTROLLER_URL`; mode changes revoke permission in either direction.

See [the main README](README.md) for calibration coordinates and the remaining
physical E-stop/restart, USB and hardware-verification work. No hardware safety
circuit is bypassed or declared tested by offline simulation.
