"""
hsv_tuner.py - live HSV sliders for one color from calibration.json.

Run from the "Raspberry Pi Code" folder (robot program NOT running, camera must be free):
    python3 hsv_tuner.py goal_purple
Color names: ball_red_low, ball_red_high, hat_yellow, hat_orange, goal_green, goal_purple

Keys (click an OpenCV window first so it has focus):
    p = print the JSON line to paste into calibration.json
    q = quit
Click on the "camera" window to print the HSV of that pixel.
"""
import sys
import os
import json
import cv2
import numpy as np
from picamera2 import Picamera2

# ---- which color, and its current values from calibration.json ----
NAME = sys.argv[1] if len(sys.argv) > 1 else "goal_purple"
PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
with open(PATH) as f:
    color = json.load(f)["hsv"][NAME]
lo, hi = color["lower"], color["upper"]

# ---- camera: same setup as the robot so the colors match ----
picam = Picamera2()
picam.configure(picam.create_video_configuration(
    main={"format": "XRGB8888", "size": (640, 480)},
    raw={"size": (1640, 1232)}))
picam.start()

# ---- sliders: start at the values already in the json ----
def nothing(x):
    pass   # createTrackbar needs a callback; we read the sliders in the loop instead

cv2.namedWindow("sliders")
for i, ch in enumerate("HSV"):
    top = 179 if ch == "H" else 255          # OpenCV hue only goes to 179
    cv2.createTrackbar(f"{ch} min", "sliders", min(lo[i], top), top, nothing)
    cv2.createTrackbar(f"{ch} max", "sliders", min(hi[i], top), top, nothing)

# ---- click a pixel to see its HSV ----
hsv = None
def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and hsv is not None:
        px, py = x // 2, y // 2              # window is shown 2x bigger than 320x240
        h, s, v = hsv[py, px]
        print(f"pixel ({px},{py}):  H {h}  S {s}  V {v}")

cv2.namedWindow("camera")
cv2.setMouseCallback("camera", on_click)

try:
    while True:
        frame = picam.capture_array()
        small = cv2.resize(frame, (320, 240))[:, :, :3]   # same size the robot uses, drop 4th channel
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        lower = np.array([cv2.getTrackbarPos(f"{ch} min", "sliders") for ch in "HSV"])
        upper = np.array([cv2.getTrackbarPos(f"{ch} max", "sliders") for ch in "HSV"])
        mask = cv2.inRange(hsv, lower, upper)

        # biggest blob area -> helps pick min_area
        kept = cv2.bitwise_and(small, small, mask=mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            big = max(contours, key=cv2.contourArea)
            cv2.drawContours(kept, [big], -1, (255, 255, 255), 1)
            cv2.putText(kept, f"biggest blob a{cv2.contourArea(big):.0f}", (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.imshow("camera", cv2.resize(small, (640, 480)))
        cv2.imshow("mask", cv2.resize(mask, (640, 480)))
        cv2.imshow("kept", cv2.resize(kept, (640, 480)))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("p"):
            print(f'"{NAME}": {{"lower": {lower.tolist()}, "upper": {upper.tolist()}}},')
        elif key == ord("q"):
            break
finally:
    picam.close()          # always free the camera, even after Ctrl-C or an error
    cv2.destroyAllWindows()