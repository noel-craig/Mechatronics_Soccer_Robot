"""
Standalone servo test - run with NOTHING else running (no robot code, motors off).
    python servo_test.py
Moves each servo through 45 / 90 / 135 / 90 using pigpio directly (hardware timed pulses).
If a servo does not move here, the problem is wiring/power/servo, not the robot code.
"""
import time
import pigpio

HOR_PIN = 12
VERT_PIN = 13

pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpiod not running - start it with: sudo pigpiod")

def angle_to_pulse(angle):
    # 0 deg = 500us, 180 deg = 2500us (same as config.py)
    return int(500 + (angle / 180) * 2000)

try:
    for name, pin in (("horizontal", HOR_PIN), ("vertical", VERT_PIN)):
        print(f"--- {name} servo on GPIO {pin} ---")
        for a in (45, 90, 135, 90):
            print(f"  {a} deg")
            pi.set_servo_pulsewidth(pin, angle_to_pulse(a))
            time.sleep(1)
        # stop pulses: servo goes limp, no jitter while holding
        pi.set_servo_pulsewidth(pin, 0)
        time.sleep(0.5)
finally:
    pi.set_servo_pulsewidth(HOR_PIN, 0)
    pi.set_servo_pulsewidth(VERT_PIN, 0)
    pi.stop()
    print("done")
