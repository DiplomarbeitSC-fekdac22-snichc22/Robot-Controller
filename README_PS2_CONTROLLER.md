# Raspberry Pi PS2 robot-arm controller

This controller talks directly to:

- the PS2 receiver through Raspberry Pi SPI0/CE0;
- the PCA9685 servo board at I2C address `0x40`.

It does not use an Arduino, LCD, cabinet buttons, or relay.

## Files

Copy these into the existing `Robot-Controller` repository:

```text
Robot-Controller/
├── ps2_manual_control.py
├── ps2_controller.py
├── run_ps2_controller.sh
├── config.py
└── config/
    ├── servos.toml
    └── ps2_controller.toml
```

`ps2_controller.py`, `config.py`, and `config/servos.toml` already exist.

## PS2 receiver connection

The existing driver uses `/dev/spidev0.0`:

| PS2 receiver label | Raspberry Pi signal | BCM GPIO | Physical pin |
|---|---|---:|---:|
| DAT | SPI0 MISO | GPIO9 | 21 |
| CMD | SPI0 MOSI | GPIO10 | 19 |
| ATT / CS | SPI0 CE0 | GPIO8 | 24 |
| CLK | SPI0 SCLK | GPIO11 | 23 |
| GND | Ground | - | 6 |
| VCC | 3.3 V | - | 1 |

Raspberry Pi GPIO uses 3.3 V logic. Do not feed 5 V logic into a GPIO pin.
Receiver boards can use different printed labels, so match the signal names,
not the wire colours.

Enable SPI through:

```bash
sudo raspi-config
```

Select `Interface Options`, then `SPI`, then enable it. Reboot if requested.

Verify the device:

```bash
ls -l /dev/spidev0.0
```

## First run: controller diagnosis only

This mode never opens the PCA9685 and cannot move a servo:

```bash
cd ~/dev/Robot-Controller
bash run_ps2_controller.sh --diagnose
```

Move both sticks and press START, SELECT, L1, and R1. The four stick values
should rest near `128` and travel toward `0` and `255`. Button presses appear
in the hexadecimal `buttons` value.

Stop with `Ctrl+C`.

## Manual arm control

Make sure no other application is controlling the PCA9685, then run:

```bash
cd ~/dev/Robot-Controller
bash run_ps2_controller.sh
```

The script starts with all configured PWM outputs disabled.

| PS2 input | Action |
|---|---|
| START | Enable or disable all configured PWM outputs |
| SELECT | Immediately disable all configured PWM outputs |
| Left stick X | Rotate base |
| Left stick Y | Move shoulder |
| Right stick Y | Move elbow |
| Right stick X | Move wrist |
| R1 | Open gripper |
| L1 | Close gripper |
| Ctrl+C | Disable outputs and exit |

If valid controller packets stop for 0.5 seconds, the program disables every
configured servo output.

## Servo configuration

All editable pulse limits, initial pulses, stick directions, dead-zone values,
and movement speeds are in:

```text
config/ps2_controller.toml
```

Pressing START sends the configured `start_pulse_us` values. These values must
describe a mechanically valid arm pose before enabling PWM.

The configured channels are:

| Joint | PCA9685 channel |
|---|---:|
| Base | 0 |
| Shoulder | 2 |
| Elbow | 4 |
| Wrist | 6 |
| Gripper | 8 |

The gripper uses the tested values:

```text
Open:   700 us
Closed: 1275 us
```
