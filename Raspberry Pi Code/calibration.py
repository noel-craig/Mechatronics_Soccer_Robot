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
DEFENSE_SPEED = CAL["defense"]["speed"]
DEFENSE_K_HEAD = CAL["defense"]["k_head"]
DEFENSE_MAX_ROT = CAL["defense"]["max_rot"]
DEFENSE_SEP_DEG = CAL["defense"]["sep_deg"]
DEFENSE_BACKUP_TIME = CAL["defense"]["backup_time"]
DEFENSE_LOOK_TIME = CAL["defense"]["look_time"]
DEFENSE_SWEEP_DEG = CAL["defense"]["sweep_deg"]

# ---- ball distance fit ----
BALL_DIST_K = CAL["ball_distance"]["k"]
BALL_CLOSE_FT = CAL["ball_distance"]["close_ft"]
BALL_SIZE = CAL["ball_distance"]["ball_size"]

# ---- escaping ir ----
ESCAPE_SPEED = CAL["escape"]["escape_speed"]
ESCAPE_TIME = CAL["escape"]["escape_time"]

# ---- camera ----
CAM_PAN_SIGN = CAL["camera"]["pan_sign"]
CAM_DEG_PER_PX = CAL["camera"]["deg_per_px"]

# ---- turning ----
TURN_K = CAL["turn"]["k"]
TURN_MAX_ROT = CAL["turn"]["max_rot"]
TURN_MIN_ROT = CAL["turn"]["min_rot"]
TURN_TOL = CAL["turn"]["tol"]

# ---- init ----
INIT_SETTLE = CAL["init"]["settle"]
INIT_LOOK_TIME = CAL["init"]["look_time"]
INIT_STEP_DEG = CAL["init"]["step_deg"]
INIT_MAX_STEPS = CAL["init"]["max_steps"]
INIT_TIMEOUT = CAL["init"]["timeout"]

# ---- search ----
SEARCH_SETTLE = CAL["search"]["settle"]
SEARCH_LOOK_TIME = CAL["search"]["look_time"]
SEARCH_STEP_DEG = CAL["search"]["step_deg"]
SEARCH_MAX_STEPS = CAL["search"]["max_steps"]
SEARCH_MEMORY = CAL["search"]["memory"]
BALL_CONFIRM_FRAMES = CAL["search"]["confirm_frames"]

# ---- approach ----
APPROACH_SPEED = CAL["approach"]["speed"]
APPROACH_MIN_SPEED = CAL["approach"]["min_speed"]
APPROACH_SLOW_FT = CAL["approach"]["slow_ft"]
APPROACH_STOP_FT = CAL["approach"]["stop_ft"]
APPROACH_K_ROT = CAL["approach"]["k_rot"]
APPROACH_MAX_ROT = CAL["approach"]["max_rot"]
APPROACH_TURN_ONLY_DEG = CAL["approach"]["turn_only_deg"]
APPROACH_LOST_TIME = CAL["approach"]["lost_time"]

# ---- get_behind ----
BEHIND_RADIUS_FT = CAL["get_behind"]["radius_ft"]
BEHIND_STRAFE_SPEED = CAL["get_behind"]["strafe_speed"]
BEHIND_MIN_STRAFE = CAL["get_behind"]["min_strafe"]
BEHIND_K_ORBIT = CAL["get_behind"]["k_orbit"]
BEHIND_K_DIST = CAL["get_behind"]["k_dist"]
BEHIND_MAX_FWD = CAL["get_behind"]["max_fwd"]
BEHIND_FF = CAL["get_behind"]["ff"]
BEHIND_K_FACE = CAL["get_behind"]["k_face"]
BEHIND_MAX_ROT = CAL["get_behind"]["max_rot"]
BEHIND_TOL = CAL["get_behind"]["tol"]
BEHIND_FACE_TOL = CAL["get_behind"]["face_tol"]
BEHIND_TIMEOUT = CAL["get_behind"]["timeout"]

# ---- attack ----
ATTACK_SPEED = CAL["attack"]["speed"]
ATTACK_K_HEAD = CAL["attack"]["k_head"]
ATTACK_MAX_ROT = CAL["attack"]["max_rot"]
ATTACK_K_STRAFE = CAL["attack"]["k_strafe"]
ATTACK_MAX_STRAFE = CAL["attack"]["max_strafe"]
ATTACK_REALIGN_DEG = CAL["attack"]["realign_deg"]
ATTACK_LOST_TIME = CAL["attack"]["lost_time"]
ATTACK_BLIND_FT = CAL["attack"]["blind_ft"]
ATTACK_BLIND_TIME = CAL["attack"]["blind_time"]
ATTACK_GOAL_BLEND = CAL["attack"]["goal_blend"]
ATTACK_TIMEOUT = CAL["attack"]["timeout"]

# ---- PK ----
PK_SPEED = CAL["pk"]["speed"]
PK_TIME = CAL["pk"]["time"]