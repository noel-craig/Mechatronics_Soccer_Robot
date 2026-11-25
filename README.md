# Mechatronics Soccer Robot

### Headless Mode Login 

**Email:** fekr9180@colorado.edu

**Password:** n2$dLzWgF+-&v25


To start on the RPi you need to get into the virtual environment:
    In Terminal:
        cd Mechatronics_Soccer_Robot
    In Terminal:
        source .venv/bin/activate
The .venv contains all of our required libraries
    If for some reason the .venv is deleted we can use requirements.txt to install all of them again (see "Rebuilding the .venv" below)

---

## Startup (every run)

1. **Power and connections**
   - Arduino Mega plugged into the Pi by USB (shows up as `/dev/ttyACM0`).
   - Motor battery on, camera ribbon and MPU6050 connected.
2. **Set the switches before starting** (they are only read once at startup):
   - **Goal switch** (GPIO 5): high = purple post on the left of the goal we score on, low = green on the left.
   - **PK switch** (GPIO 6): on = penalty kick mode (drives straight, then normal play).
3. **Open a terminal on the Pi** (headless: VNC or `ssh -X` if you want the debug camera windows).
4. **Start the servo daemon** (once per boot, so the camera servos don't jitter):
   ```
   sudo pigpiod
   ```
   To start it automatically at every boot instead: `sudo systemctl enable --now pigpiod`
5. **Activate the venv and go to the code folder:**
   ```
   cd Mechatronics_Soccer_Robot
   source .venv/bin/activate
   cd "Raspberry Pi Code"
   ```
6. **Place the robot on the field facing roughly toward the goal we score on**, then run:
   ```
   python3 philfixeverything.py
   ```
7. **Don't touch the robot until `GYRO BIAS: ...` prints.** The gyro calibrates for 2 seconds at startup. If it prints `WARNING: ... Robot Moved`, stop and restart.
8. **Stop with Ctrl-C.** This sends `<STOP>` to the Arduino and closes the camera and GPIO cleanly.

### Settings to check before a run
| Where | Setting | Testing | Competition |
|---|---|---|---|
| `philfixeverything.py` | `DEBUG_VIEW` | `True` | `False` |
| `philfixeverything.py` | `SHOW_MASKS` | `True` (HSV tuning) | `False` |
| `philfixeverything.py`, `main()` | `state.number` | whatever state you're testing | `-1` (Init) |

All tuning numbers (speeds, gains, HSV colors, timeouts) are in `Raspberry Pi Code/calibration.json`. Edit and restart the program; no code changes needed. The full test procedure is in [TEST_PLAN.md](TEST_PLAN.md).

### Quick check after code changes (no hardware needed)
```
python3 -c "import calibration, states; print('ok')"
```
Run it from the `Raspberry Pi Code` folder. It catches syntax errors and missing calibration.json keys.

### Arduino
The Arduino code (`Arduino Code/Mechatronics`) is a PlatformIO project. Open that folder in VS Code with the PlatformIO extension and use **Upload**. It only needs re-uploading when the Arduino code changes.

---

## Rebuilding the .venv

Only needed if `.venv` gets deleted or broken.

```
sudo apt update
sudo apt install -y python3-picamera2 pigpio
cd Mechatronics_Soccer_Robot
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

- `picamera2` comes from apt, not pip. `--system-site-packages` lets the venv see it.
- Also enable the camera and I2C (needed for the MPU6050) if this is a fresh Pi: `sudo raspi-config` → Interface Options.
- Serial access: if `/dev/ttyACM0` gives a permission error, run `sudo usermod -aG dialout $USER` and log out and back in.