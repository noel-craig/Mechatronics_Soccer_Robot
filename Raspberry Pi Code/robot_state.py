"""
Shared runtime state for the robot.

Everything that CHANGES while the robot is running lives here (vision results, heading,
switch states). Every file does:

    from robot_state import rs

and reads/writes rs.<name>. Because there is only one rs object, everyone sees the same
live values, and no `global` statements are needed.

Tuning numbers that do NOT change while running go in calibration.json instead.
"""


class RobotState:
    def __init__(self):
        # ---- ball (from process_frame) ----
        self.found_ball = False
        self.ball_x = 0
        self.ball_y = 0
        self.ball_r = 0

        # ---- enemy hat ----
        self.found_hat = False
        self.hat_x = 0
        self.hat_y = 0

        # ---- goal posts ----
        self.found_green_goal = False
        self.green_x = 0
        self.green_y = 0
        self.found_purple_goal = False
        self.purple_x = 0
        self.purple_y = 0

        # ---- heading (from imu_thingy_for_stuff) ----
        self.angle_z = 0 # integrated gyro heading in degrees
        self.goal_heading = 90 # heading that faces the goal we score on, set in the init state

        # ---- switches (read in initialize) ----
        self.goal_state = 0 # 1 = purple left, 0 = green left
        self.pk_state = 0 # 1 = penalty kick mode
        self.hat_state = 0 # 1 = enemy wears yellow hat, 0 = orange hat

        # ---- IR border sensors ----
        # A 1 means this corner is over the white border
        self.ir_fl, self.ir_fr, self.ir_bl, self.ir_br = 0,0,0,0

        # ---- Camera Settings ----
        self.cam_pan = 90

    def reset_vision(self):
        """clear the found flags at the start of each frame"""
        self.found_ball = False
        self.found_hat = False
        self.found_green_goal = False
        self.found_purple_goal = False


rs = RobotState()
