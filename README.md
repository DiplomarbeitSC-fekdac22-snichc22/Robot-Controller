# PIA Robot Controller for Raspberry Pi

This is the Python translation of `FINAL_PCA 3.ino`. It preserves the three
operating modes, 60-second game timer, long-press test mode, PS2 stick mapping,
servo limits, gripper buttons, LCD text and active-low relay behavior.

## Files

- `main.py`: state machine and main control loop
- `config.py`: pins, timing, servo channels, limits and initial positions
- `hardware.py`: PCA9685, LCD, GPIO buttons and relay
- `ps2_controller.py`: direct PS2-controller SPI communication

## Raspberry Pi wiring

All GPIO numbers in `config.py` use **BCM numbering**.

| Device signal | Raspberry Pi connection |
| --- | --- |
| Start button | GPIO17 to GND |
| Stop button | GPIO27 to GND |
| Test button | GPIO22 to GND |
| LED relay input | GPIO23 |
| PCA9685 + LCD SDA | GPIO2 / pin 3 |
| PCA9685 + LCD SCL | GPIO3 / pin 5 |
| PS2 DAT | GPIO9 / SPI0 MISO / pin 21 |
| PS2 CMD | GPIO10 / SPI0 MOSI / pin 19 |
| PS2 CLK | GPIO11 / SPI0 SCLK / pin 23 |
| PS2 ATT | GPIO8 / SPI0 CE0 / pin 24 |

Use a common ground. Power the servos from a suitable separate supply, not from
the Raspberry Pi, and connect the supply ground to the Pi/PCA9685 ground. Ensure
that no signal going into a Raspberry Pi GPIO exceeds 3.3 V; add level shifting
if the PS2 receiver or relay board uses 5 V logic.

## Setup and run

Enable I2C and SPI in `raspi-config`, then reboot. On Raspberry Pi OS, install
the GPIO backend and I2C tools:

```bash
sudo apt update
sudo apt install python3-venv python3-lgpio i2c-tools
```

Create an environment and install the Python dependencies:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 main.py
```

If your pins, I2C addresses or PCA9685 channels differ, change only `config.py`.
The unusual 230-degree maximum for the base was retained exactly: like the
Arduino sketch, it produces a PWM pulse beyond the nominal 180-degree mapping.
Confirm that this is safe for the actual base servo before moving the robot.

