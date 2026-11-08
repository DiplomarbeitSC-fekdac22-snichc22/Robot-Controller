# PIA Robot Controller for Raspberry Pi

The deployed `robot-arm-controller.service` runs `run_ps2_controller.sh`, which
starts `ps2_manual_control.py`. It controls the PS2 receiver, PCA9685, cabinet
buttons, permission LED and LCD. `main.py` / `hardware.py` / `config.py` are
older entry points with different wiring; they are not the deployed workflow.
See [the PS2 guide](README_PS2_CONTROLLER.md) for the current controls and wiring.

## Operating workflow

The permission/display sequence follows Markus's `Ablaufbeschreibung Roby LWS.docx`.
Its hardware findings remain separate from the software behaviour below.

The controller starts in **manual mode with PWM disabled**. Connect the joystick,
release its buttons and centre the sticks. With the arm physically matching the
stored pulse position, press PS2 START to confirm that position; this confirmation
itself sends no servo command. Then press the green cabinet button for a
90-second movement permission, or the configured white button for untimed setup
permission. Setup permission lasts until cancellation. Position confirmation is
required again after stopping, disconnecting, or changing operating mode.

| Input / event | Result |
| --- | --- |
| Green button | Timed permission, 90 seconds by default |
| White setup button | Untimed setup permission, when its GPIO is configured |
| Red button / D-pad DOWN | Revoke permission and disable PWM immediately |
| Timer expiry | Revoke permission and disable PWM; show `Game over` |
| Joystick disconnect | Revoke permission and disable PWM |
| PS2 SELECT / mode request | Switch manual/automatic, revoke permission and disable PWM |
| PS2 START | Confirm actual position while stopped with neutral controls |
| D-pad UP | Toggle PWM only within an already granted permission; cannot grant permission |

Disabling PWM with D-pad UP also revokes permission; starting again requires a
fresh position confirmation and cabinet-button grant.

The green LED is on only when movement permission and PWM are both active.
The LCD shows the requested standby, remaining-time, `Einstellbetrieb`,
`Game over`, and `Abbruch der Freigabe` states. Expiry/cancellation messages last
about 10 seconds before standby. Held stop inputs take priority over start.

Expiry, cancellation and reconnection do not open the gripper, return the arm
home, or automatically re-arm it. No recovery motion is assumed safe from an
unknown physical position. Verify the physical arm position before confirming
stored pulses; this controller has no joint-position sensors.

Joystick motion uses a short velocity ramp within the existing per-axis speed
limits; gripper commands move toward their target in small steps. Existing joint
limits and the glass-door guard remain active. Actual fluid motion and mechanics
still need testing on the robot.

## Disconnected testing

Simulation uses the same controller and permission logic without importing GPIO,
opening I2C, or writing physical PWM:

```bash
python ps2_manual_control.py --simulate
```

Use its interactive commands to confirm position, grant green/setup permission,
move a stick, cancel, change modes, and simulate connection/E-stop changes.
Inspect the simulated pulse outputs and permission status. Run automated tests:

```bash
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s tests -v
```

`bash run_ps2_controller.sh --diagnose` is a separate **Pi-only** joystick input
test that does not open the PCA9685.

## Configuration and mode connection

`config/ps2_controller.toml` retains the measured servo channels, pulse limits,
directions and maximum speeds. Its `[operation]` section configures permission
and display durations, motion ramping, optional setup/E-stop GPIO inputs, and
the local control API. The white-button pin and E-stop contact wiring are unknown,
so neither pin is invented or enabled by default. Configure them only from the
actual cabinet wiring. Optional E-stop feedback supports a verified supervised
normally-closed contact to GND only: configure its pin together with
`emergency_stop_healthy_pressed = true`. An open/broken contact then blocks
permission; the unverified inverse polarity is rejected.

The controller's local API listens on loopback port 8765 by default. `GET /status`
reports its mode and permission; `POST /mode` accepts `manual` or `automatic`.
These routes cannot command motion or grant permission. Set the backend's
`ROBOT_CONTROLLER_URL` to this local controller address so the webpage's mode
selection changes the running controller. When the connection is unavailable,
the backend reports the missing controller and keeps movement blocked.

Automatic mode blocks joystick/gripper motion. Camera pickup remains unavailable
until verified target/grasp validation, existing kinematics execution, controller
permission arbitration and hardware interlocks are connected. Mode selection
never starts a movement. Do not run another PCA9685 writer alongside this process.

Kinematics expects `robot_base` millimetres: X forward/depth, Y downward from the
workcell top, Z right/lateral; origin at the roof mounting plate centre. Shelf
observations need measured alignment with that frame. Its existing
`api.RobotController.run_pick_and_place(x_mm, y_mm, z_mm)` validates the complete
motion plan; no kinematics is duplicated here. A triangulated bounding-box centre
is not a verified grasp point.

## Hardware checks still required

Software revocation and optional E-stop feedback do not establish that the
physical emergency-stop circuit cuts actuator power correctly. Verify contact
polarity, broken-wire response, 5 V/servo power behaviour, controller restart,
and fresh-position confirmation/start after releasing E-stop. The previously
reported electrical restart problem is not claimed fixed by software.

On the Pi, first verify stopped PWM and the actual pose with the controller
owner. Then test green/white permission, red cancellation, the 90-second timeout,
LED/LCD states, joystick loss/reconnection, and manual/automatic switching.
Confirm that cancellation never initiates home/gripper movement. Recheck all
joint limits and motion smoothness before attempting camera pickup.

The keyboard/mouse USB limitation needs a physical connection solution; no
software change creates another USB port. Deployment files and system services
were not changed by this controller work.
