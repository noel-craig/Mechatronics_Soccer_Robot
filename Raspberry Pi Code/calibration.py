"""
Loads calibration.json once at startup and exposes the values as constants.

No hardware imports here, so this also works on a laptop (e.g. for an HSV tuning script).
To retune: edit calibration.json and restart the program.
"""
import json
import os
import numpy as np

# find the json next to this file, no matter what folder the program was started from
_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")

with open(_PATH) as f:
    CAL = json.load(f)


def _hsv(name):
    """returns (lower, upper) numpy arrays for a color in calibration.json"""
    c = CAL["hsv"][name]
    return np.array(c["lower"]), np.array(c["upper"])


# ---- colors ----
# red wraps around the hue circle so the ball needs two ranges (0-5 and 170-180)
low_lower_red, low_upper_red = _hsv("ball_red_low")
up_lower_red, up_upper_red = _hsv("ball_red_high")
lower_yellow, upper_yellow = _hsv("hat_yellow")
lower_orange, upper_orange = _hsv("hat_orange")
lower_green, upper_green = _hsv("goal_green")
lower_purple, upper_purple = _hsv("goal_purple")

# ---- minimum blob sizes (px area) ----
min_ball_area = CAL["min_area"]["ball"]
min_hat_area = CAL["min_area"]["hat"]
min_goal_area = CAL["min_area"]["goal"]

# ---- defense ----
DEFENSE_GOAL_PX = CAL["defense"]["goal_px"]
DEFENSE_HEADING_TOL = CAL["defense"]["heading_tol"]

# ---- ball distance fit ----
BALL_DIST_INTERCEPT = CAL["ball_distance"]["intercept_ft"]
BALL_DIST_SLOPE = CAL["ball_distance"]["slope_ft_per_px"]
BALL_CLOSE_FT = CAL["ball_distance"]["close_ft"]
BALL_SIZE = CAL["ball_distance"]["ball_size"]

# ---- escaping ir ----
ESCAPE_SPEED = CAL["escape"]["escape_speed"]
ESCAPE_TIME = CAL["escape"]["escape_time"]