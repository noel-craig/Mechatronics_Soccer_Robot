import random
import numpy as np
from robot_state import rs # shared live data: rs.found_ball, rs.ball_x, rs.angle_z, ...
import calibration as cal # tuning numbers from calibration.json
import time

# NOTE: do NOT import philfixeverything here (circular import).
# The main file passes itself in as `robot`. Use self.r.<function>() for hardware
# functions (process_frame, send_command, drive_update, ...) and self.r.ser for serial.

"""The state machine states are outlined as:
States:
# -2: Defense mode. switch to 1 when we find the ball
# -1: initialize (find goal, set gyro)
# 0: Finding Ball (look around + back up till ball found then state 1)
# 1: Going To Ball (angle drive until distance is small then state 2) (if lost go to state 0)
# 2: Rotating for kick (center goal in frame with rotation then got to state 3)
# 3: Driving into goal (look up at goal and drive straight if other robot seen kick then go to state 0)
# 4: Zeroing angle (entered if IR sensor tripped from state 0 or 1) 
    i dont see how this helps, we should check IR in all states, if any trigger drive away from the wall and return to that state to keep going.
    the time wasted trying to reorient is wasted if the ball bounces off the wall and goes back toward the mid field
(rotate about triped wheel until 2 sensors trip then enter 0)
do we want to remove state 0 of find ball because defense should be what we do anyway? 
"""

def wrap_180(angle):
    """
    Takes any angle and returns the same direction expressed in -180 to 180
    """
    return (angle + 180) % 360 - 180

class State_Machine():
    def __init__(self, robot, state_number = -1):
        self.r = robot # the main philfixeverything module
        self.number = state_number
        self.spin_dir = random.choice([-90, 90]) # random left or right direction might still be too fast
        self.speed = 0
        self.angle = 0
        self.rotation = 0
        self.search_start_angle = None
        self.prev_number = state_number
        self.state_start_time = time.time() # used for timeouts in states
        self.escape_until = 0
        self.escape_angle = 0

    def set_state(self, new_number):
        """
        Used to handoff state transitions.
        """
        if new_number == self.number:
            return # avoid time spent on this if still in same state
        print(f"STATE {self.number} -> {new_number}")
        self.search_start_angle = None # Mostly used for tracking rotations in init
        self.prev_number = self.number # in case we need to get back to the state from an IR Read
        self.state_start_time = time.time() # used for timeouts in some methods
        self.number = new_number # finally set state to new state number

    def check_escape(self):
        """
        Check if the robot needs to escape the white line using the IR sensors
        Called once per tick from the main loop.
        Returns True if we are escaping this tick (main loop should skip the state),
        False otherwise.
        """
        now = time.time()
        already_escaping = now < self.escape_until
        fl, fr, bl, br = rs.ir_fl, rs.ir_fr, rs.ir_bl, rs.ir_br

        if fl or fr or bl or br:
            # each tripped corner pushes us away from itself
            forward_push = (bl+br) - (fl + fr) # + = move forward
            left_push = (fr + br) - (fl + bl) # - = move left
            if forward_push == 0 and left_push == 0:
                # diagonal pair or all four: no clear direction, just stop
                self.r.send_command(0,0,0)
                self.r.drive_update()
                return True
            self.escape_angle = np.degrees(np.arctan2(left_push, forward_push))
            self.escape_until = now + cal.ESCAPE_TIME
            if not already_escaping:
                print(f"ESCAPE {self.escape_angle:.0f}")

        if now < self.escape_until:
            # still escaping: slide away, no rotation
            self.r.send_command(cal.ESCAPE_SPEED, self.escape_angle, 0)
            self.r.drive_update()
            return True

        return False

    def bearing_to(self, x_px):
        """
        Absolute heading (same frame as rs.angle_z, + = left) of something
        seen at pixel column x_px in the current camera frame.
        """
        pan_offset = cal.CAM_PAN_SIGN * (rs.cam_pan - 90) # where the camera points vs robot front
        pixel_offset = -(x_px - 160) * cal.CAM_DEG_PER_PX # right of center = negative
        return wrap_180(rs.angle_z + pan_offset + pixel_offset)

    def turn_toward(self, target_heading):
        """
        One tick of a proportional turn toward an absolute heading (+ = left / CCW).
        Returns True when within TURN_TOL (and the robot is stopped), False while still turning.
        """
        error = wrap_180(target_heading - rs.angle_z)
        if abs(error) < cal.TURN_TOL:
            self.r.send_command(0,0,0)
            self.r.drive_update(force=True)
            return True

        rot = cal.TURN_K * abs(error)
        rot = max(cal.TURN_MIN_ROT, min(cal.TURN_MAX_ROT, rot))
        self.r.send_command(0, 0, rot if error > 0 else -rot)
        self.r.drive_update()
        return False

    def defense(self):

        """
        state.number = -2
        Defense: face the goal we score on (with +/-90 deg of GOAL_HEADING),
            back-up toward our own goal until the opposing goal posts look close enough together,
            then wait until we see the ball.
        """
        self.r.process_frame(["ball", "goal"]) # process if we can see the ball or the other goal
        # how far off we are from GOAL_HEADING, wrapped to -180..180
        heading_error = wrap_180(rs.goal_heading - rs.angle_z)
        if rs.found_ball:
            print("defense: found ball")
            self.set_state(1)
        elif abs(heading_error) > cal.DEFENSE_HEADING_TOL:
            # facing the wrong way, rotate toward GOAL_HEADING before backing up
            print(f"defense: wrong way, heading err {heading_error:.1f}, rotating")
            self.r.send_command(0, 0, 90 if heading_error > 0 else -90) # check rotation sign for direction
            self.r.drive_update()
        elif rs.found_green_goal and rs.found_purple_goal:
            goal_px = abs(rs.green_x - rs.purple_x) # gets smaller the further we are from the goal
            print(f"defense: goal px sep {goal_px:.0f} (target {cal.DEFENSE_GOAL_PX}), heading err {heading_error:.1f}")
            if goal_px > cal.DEFENSE_GOAL_PX:
                # still too close to their goal, keep backing up
                self.r.send_command(150, 180, 0) # 180 = straight back, check angle
                self.r.drive_update()
            else:
                # at our end of the field, stop and wait for the ball
                print("defense: in position")
                self.r.send_command(0, 0, 0)
                # can go into wiggle
                self.r.drive_update()
        else:
            # facing roughly right but cant see both posts, rotate toward GOAL_HEADING to find them
            print(f"defense: goals not both seen, heading err {heading_error:.1f}")
            self.r.send_command(0, 0, 60 if heading_error > 0 else -60) # check rotation sign
            self.r.drive_update()

    def init(self):
        """
        self.number = -1
        Initialize State: We need to find both of the goal colors at the beginning of the match
        """

        self.r.process_frame(["goal"]) # we're only looking for the opposing goal and not the ball at this point

        if self.search_start_angle is None:
            self.search_start_angle = rs.angle_z
        
        if rs.found_green_goal and rs.found_purple_goal:
            print("found both goals")
            goal_px = abs(rs.green_x - rs.purple_x) # gets smaller the further we are from the goal
            print(f"defense: goal px sep {goal_px:.0f} (target {cal.DEFENSE_GOAL_PX})")
        
            if rs.goal_state: # purple left goal read from the switch
                if rs.green_x - rs.purple_x > 0: # green is further right on camera than purple
                    heading = rs.angle_z # save heading when we see the goal we want
                    print(f"heading 4 goal {heading}")
                    rs.goal_heading = heading
                    self.r.send_command(0,0,0)
                    self.r.ser.write("<STOP>".encode())
                    print("Going to state 0")
                    self.set_state(0) # go to find ball
                else: # need to turn around we see the wrong goal
                    self.r.send_command(0,0,self.spin_dir)
                    self.r.drive_update()

            else: # green left goal (from goalstateswitch)
                if rs.purple_x - rs.green_x > 0:
                    heading = rs.angle_z # save the heading 
                    print(f"heading 4 goal {heading}")
                    rs.goal_heading = heading
                    self.r.send_command(0,0,0)
                    self.r.ser.write("<STOP>".encode())                 
                    self.set_state(0) # go to find ball
                else: # need to turn around we see the wrong goal
                    self.r.send_command(0,0,self.spin_dir)
                    self.r.drive_update()
        else:
            # we need to keep track of revolutions and then switch to defensive mode after a certain number of revolutions
            revolutions = abs(rs.angle_z - self.search_start_angle) / 360
            if revolutions >= 3:
                print("no goals after 3 revs, going to defense")
                self.r.send_command(0,0,0)
                self.r.drive_update()
                self.set_state(-2) # go to defense mode
            else:
                print(f"no goals, spinning ({revolutions:.2f} rev)")
                self.r.send_command(0,0,self.spin_dir)
                self.r.drive_update()

    def find_ball(self):
        """
        Finding Ball (spin around until ball found, then state 1)
        check ir
        check camera for ball
        cam scan
        if found_ball true, change to state 1  
        if ball too damn close, state 2
        """
        self.r.process_frame(["ball", "goal"])
        if rs.found_ball:
            self.set_state(1)
        # backup and cam scan
        self.r.send_command(-100, 0, 0)
        self.r.drive_update()
        self.r.cam_sweep(0,45,135)

        #if found_green_goal and found_purple_goal:

        # if backed up:
        #     state = -2
        # add look with servo

        # if found_ball:
        #     print(f"found ball x:{ball_x:.0f} r:{ball_r:.0f}")
        #     if ball_r > 88: # closer than ~2ft, skip straight to lining up the kick
        #         state = 2
        #     else:
        #         state = 1
        # else:
        #     # spin in place to look for the ball
        #     print("looking for ball")
        #     send_command(0, 0, spin_dir)
        #     drive_update()
        #     # option: stop and sweep camera instead of spinning robot
        #     # send_command(0, 0, 0); drive_update()
        #     # cam_sweep(0, 30, 150); rotate_cam(90)
        #     # option: if spun a full circle (use angle_z) and no ball, back up or go to defense
        #     # if abs(angle_z - search_start_angle) > 360:
        #     #     state = -2
    # This is old code for chasing the ball prolly still good and for state 1

    def go_to_ball(self):
        """
        Going To Ball (angle drive until distance is small then state 2) (if lost go to state 0)

        check ir
        check camera update for ball location
        do drive to ball code,
        if found ball false, go to state 0 
        process_frame(["ball"])
        """
        self.r.process_frame(["ball"])

        if rs.found_ball:
            predicted_distance = cal.BALL_DIST_K / np.sqrt(rs.ball_area) # distance in ft.
            hor_error = rs.ball_x - 160  # hor_error is horizontal error from center screen pixel
            hor_pos = (cal.BALL_SIZE / (2 * rs.ball_r)) * hor_error
            if predicted_distance > cal.BALL_CLOSE_FT: # if the ball is greater than 1ft away we can determine angle to the ball
                self.angle = -np.arctan2(hor_pos, predicted_distance) * 180 / np.pi
            else:
                self.angle = 0 # if we're within 1 ft of the ball, it pretty much takes up the FOV, so set the angle to 0 and CHANGE STATE TO FIND THE GOAL with cam
            self.speed = 120
            self.r.send_command(self.speed, self.angle, self.rotation)
            self.r.drive_update()
            # print(f"dist {predicted_distance:.2f}ft  angle {angle:.1f}  cmd {next_command}")
            # if predicted_distance <= 2.1: # close enough, line up the kick
            #     send_command(0, 0, 0)
            #     drive_update()
            #     state = 2

        else:
            self.r.send_command(0,0,0)
            self.r.drive_update()
            print("lost ball")
            self.set_state(0) # go back to find the ball now that we lost it

    def PK(self):
        """
        Penalty Kick Mode
        Use one of the active switch states to put the robot into PK mode
        This will drive forward for a set amount of time to hit the ball and then stop
        """
        self.r.send_command(200, 0, 0)
        self.r.drive_update()

    def rotate_for_kick(self):
        """
        Rotating for kick (center goal in frame with rotation then got to state 3)

        check ir
        check camera for ball and goal pieces ()
            if one goal piece check with gyro for direction of our desired goal
        if found ball and both goal, drive to put goal intersection 
        
        """
        self.r.process_frame(["ball","goal"])

        # WE MAY NEED TO REMOVE THIS FIRST STATEMENT.  WHEN THE BALL IS CLOSE IT TAKES UP FOV
        if not rs.found_ball:
            print("lost ball while lining up")
            self.set_state(0)
        elif rs.found_green_goal and rs.found_purple_goal:
            goal_center_x = (rs.green_x + rs.purple_x) / 2
            goal_error = goal_center_x - rs.ball_x  # want goal center lined up behind the ball
            ball_error = rs.ball_x - 160            # keep ball centered in frame
            print(f"goal err {goal_error:.0f}  ball err {ball_error:.0f}")
            if abs(goal_error) < 15 and abs(ball_error) < 20:
                self.r.send_command(0, 0, 0)
                self.r.drive_update()
                self.set_state(3)
            else:
                # orbit the ball: strafe sideways while rotating to keep ball centered
                strafe_angle = 90 if goal_error < 0 else -90   # check sign, negative angle = right
                rotation = -0.5 * ball_error                   # check sign
                self.r.send_command(120, strafe_angle, rotation)
                self.r.drive_update()
        else:
            # cant see both goal posts, use gyro to turn toward GOAL_HEADING
            heading_error = wrap_180(rs.goal_heading - rs.angle_z)
            print(f"no goal in view, heading err {heading_error:.1f}")
            strafe_angle = 90 if heading_error > 0 else -90    # check sign
            self.r.send_command(120, strafe_angle, -0.5 * (rs.ball_x - 160))
            self.r.drive_update()

    def drive_into_goal(self):
        """
        Driving into goal (drive straight at goal, kick if enemy hat seen, then state 0)
        """
        self.r.tilt_cam(110) # look up at goal, check angle
        self.r.process_frame(["ball", "goal", "hat"])
        if rs.found_green_goal and rs.found_purple_goal:
            goal_center_x = (rs.green_x + rs.purple_x) / 2
            rotation = -0.5 * (goal_center_x - 160) # steer toward goal center, check sign
            self.r.send_command(255, 0, rotation)
        else:
            # hold gyro heading
            heading_error = wrap_180(rs.goal_heading - rs.angle_z)
            self.r.send_command(255, 0, 2 * heading_error) # check sign
        self.r.drive_update()
        print(f"driving to goal cmd {self.r.next_command}  hat {rs.found_hat}")
        if rs.found_hat:
            print("enemy seen, kick")
            # ser.write("<KICK>".encode()) # arduino does not have a KICK command yet
            self.r.tilt_cam(90)
            self.set_state(0)
        # if not rs.found_ball: (ball may be hidden under camera when tilted up)
        #     self.r.tilt_cam(90)
        #     self.number = 0