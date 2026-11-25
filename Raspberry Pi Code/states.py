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

def clamp(x,lo,hi):
    """limit x to the range lo -> hi"""
    return max(lo, min(hi,x))

class State_Machine():
    def __init__(self, robot, state_number = -1):
        self.r = robot # the main philfixeverything module
        self.number = state_number
        self.spin_dir = random.choice([-90, 90]) # random left or right direction might still be too fast
        self.speed = 0
        self.angle = 0
        self.rotation = 0
        self.prev_number = state_number
        self.state_start_time = time.time() # used for timeouts in states
        self.escape_until = 0
        self.escape_angle = 0
        self.phase = "look"
        self.phase_start = time.time()
        self.steps = 0
        self.turn_target = 0
        self.search_dir = 1
        self.ball_seen_count = 0
        self.orbit_dir = 1 # +1 = orbit CCW (strafe right), -1 = orbit CW (strafe left)

    def set_state(self, new_number):
        """
        Used to handoff state transitions.
        """
        if new_number == self.number:
            return # avoid time spent on this if still in same state
        print(f"STATE {self.number} -> {new_number}")
        self.phase = "look"
        self.phase_start = time.time()
        self.steps = 0
        self.turn_target = 0
        self.search_dir = 1
        self.ball_seen_count = 0
        self.orbit_dir = 1
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
        state -2: fall back and guard our goal.
        face their goal -> back up toward ours (holding heading) until their posts
        look DEFENSE_SEP_DEG apart (or time runs out) -> guard: sweep +/- SWEEP_DEG.
        Ball confirmed in any phase -> approach (1).
        """
        now = time.time()

        # every tick, every phase: one frame for ball + posts
        if self.look_for_ball(["goal"]):
            print(f"defense: ball at bearing {rs.ball_bearing:.1f} -> approach")
            self.set_state(1)
            return
        seen = self.refine_goal_heading()

        if self.phase == "look":
            self._set_phase("face")
        if self.phase == "face":
            if self.turn_toward(rs.goal_heading):
                self._set_phase("backup")

        elif self.phase == "backup":
            # posts in view and far enough away? we're home <3
            if seen is not None:
                sep = abs(rs.purple_x - rs.green_x) * cal.CAM_DEG_PER_PX
                if sep < cal.DEFENSE_SEP_DEG:
                    print(f"defense: in position (posts {sep:.0f} deg apart)")
                    self.r.send_command(0,0,0)
                    self.r.drive_update(force=True)
                    self._set_phase("guard")
                    return

            # safety net to not back up forever
            if now - self.phase_start > cal.DEFENSE_BACKUP_TIME:
                print("defense: backed up long enough >.<")
                self.r.send_command(0,0,0)
                self.r.drive_update(force = True)
                self._set_phase("guard")
                return
            # back straight up, gyro holding us facing their goal
            err = wrap_180(rs.goal_heading - rs.angle_z)
            rot = clamp(cal.DEFENSE_K_HEAD * err, -cal.DEFENSE_MAX_ROT, cal.DEFENSE_MAX_ROT)
            self.r.send_command(cal.DEFENSE_SPEED, 180, rot)
            self.r.drive_update()

        elif self.phase == "guard":
            # sit still and watch
            self.r.send_command(0,0,0)
            self.r.drive_update()
            if now - self.phase_start > cal.DEFENSE_LOOK_TIME:
                #alternate left/right of the goal heading
                self.steps += 1
                side = 1 if self.steps % 2 else -1
                self.turn_target = wrap_180(rs.goal_heading + side * cal.DEFENSE_SWEEP_DEG)
                self._set_phase("sweep")

        elif self.phase == "sweep":
            if self.turn_toward(self.turn_target):
                self._set_phase("guard")

    def _set_phase(self, phase):
        """switch Init's sub-phase and restart its timer"""
        print(f"  phase: {self.phase} -> {phase}")
        self.phase = phase
        self.phase_start = time.time()

    def init(self):
        """
        state -1: find the goal we score on and save rs.goal_heading
        Phases: look(stopped) -> turn 45deg -> look ... -> face goal -> state 0
        """
        now = time.time()

        # overall safety net. never get stuck in init
        if self.phase != "face" and now - self.state_start_time > cal.INIT_TIMEOUT:
            print("init: timeout, assuming start heading faces goal")
            rs.goal_heading = 0
            self._set_phase("face")
        if self.phase == "look":
            # let the robot finish coasting before trusting the camera
            if now - self.phase_start < cal.INIT_SETTLE:
                self.r.send_command(0,0,0)
                self.r.drive_update()
                return

# PUT THESE LINES BACK IN ONCE I CAN SENSE A GOAL
            # self.r.process_frame(["goal"])
            # seen = self.goal_bearing()
            # if seen is not None:
            #   rs.goal_heading = seen
            #   print(f"init: goal found, heading {rs.goal_heading:.1f}")
            #   self._set_phase("face")
            #   return
            # wrong goal: ignore it and keep looking
# =========================== TAKE THIS OUT ONCE I HAVE A GOAL TO LOOK AT =======
            self.r.process_frame(["ball"])
            if rs.found_ball:
                rs.goal_heading = self.bearing_to(rs.ball_x)
                print(f"init: (ball test) heading {rs.goal_heading:.1f}")
                self._set_phase("face")
                return
# ===============================================================================
            
            # looked long enough without finding it -> turn another step (or give up)
            if now - self.phase_start > cal.INIT_SETTLE + cal.INIT_LOOK_TIME:
                if self.steps >= cal.INIT_MAX_STEPS:
                    print("init: full circle, no goal, assuming start heading")
                    rs.goal_heading = 0
                    self._set_phase("face")
                else:
                    direction = 1 if self.spin_dir > 0 else -1
                    self.turn_target = wrap_180(rs.angle_z + direction * cal.INIT_STEP_DEG)
                    self.steps += 1
                    self._set_phase("turn")
        elif self.phase == "turn":
            if self.turn_toward(self.turn_target):
                self._set_phase("look")
        elif self.phase == "face":
            if self.turn_toward(rs.goal_heading):
                self.set_state(0)

    def look_for_ball(self, extra = ()):
        """
        Grab a frame, update ball memory.  Returns True once the ball has been seen
        in BALL_CONFIRM_FRAMES consecutive frames.
        """
        self.r.process_frame(["ball", *extra])
        if rs.found_ball:
            self.ball_seen_count += 1
            rs.ball_bearing = self.bearing_to(rs.ball_x)
            rs.ball_dist = cal.BALL_DIST_K / np.sqrt(rs.ball_area)
            rs.ball_seen_time = time.time()
        else:
            self.ball_seen_count = 0
        return self.ball_seen_count >= cal.BALL_CONFIRM_FRAMES

    def find_ball(self):
        """
        state 0: find the ball by turning in steps and looking while stopped.
        First step goes toward where the ball was last seen (if recent).
        Full circle with no ball -> defense (-2).  Ball confirmed -> approach (1).
        """
        now = time.time()

        if self.phase == "look":
            # let the robot stop coasting before trusting the camera
            if now - self.phase_start < cal.SEARCH_SETTLE:
                self.r.send_command(0,0,0)
                self.r.drive_update()
                return
            if self.look_for_ball():
                print(f"search: ball at bearing {rs.ball_bearing:.1f}")
                self.set_state(1)
                return
            if now - self.phase_start > cal.SEARCH_SETTLE + cal.SEARCH_LOOK_TIME:
                if self.steps >= cal.SEARCH_MAX_STEPS:
                    print("search: full circle, no ball -> defense")
                    self.set_state(-2)
                    return
                if self.steps == 0:
                    # pick a directiononce, at the first step
                    if now - rs.ball_seen_time < cal.SEARCH_MEMORY:
                        # seen recently: turn toward where it went
                        side = wrap_180(rs.ball_bearing - rs.angle_z)
                        self.search_dir = 1 if side > 0 else -1
                        print(f"search: last seen {'left' if side > 0 else 'right'}, turning that way")
                    else:
                        self.search_dir = 1 if self.spin_dir > 0 else -1
                self.turn_target = wrap_180(rs.angle_z + self.search_dir * cal.SEARCH_STEP_DEG)
                self.steps += 1
                self._set_phase("turn")

        elif self.phase == "turn":
            if self.turn_toward(self.turn_target):
                self._set_phase("look")

    def go_to_ball(self):
        """
        state 1: drive at the ball, rotating to keep facing it.
        Steers on rs.ball_bearing vs gyro so a missed frame doesn't matter.
        Close enough -> get behind (2).  Lost too long -> search(0)
        """
        now = time.time()
        self.look_for_ball() # updates rs.ball_bearing /ball_dist/ball_seen_time when seen

        # lost it?
        if now - rs.ball_seen_time > cal.APPROACH_LOST_TIME:
            print("approach: lost ball -> search")
            self.set_state(0)
            return

        # close enough? (only trust distance form a frame that saw it this tick)
        if rs.found_ball and rs.ball_dist < cal.APPROACH_STOP_FT:
            print(f"approach: at ball ({rs.ball_dist:.1f} ft) -> get behind")
            self.r.send_command(0,0,0)
            self.r.drive_update(force=True)
            self.set_state(2)
            return

        # steer: how far is the ball from where we're facing (+ = ball is left)
        error = wrap_180(rs.ball_bearing - rs.angle_z)
        rot = cal.APPROACH_K_ROT * error
        rot = max(-cal.APPROACH_MAX_ROT, min(cal.APPROACH_MAX_ROT, rot))

        if abs(error) > cal.APPROACH_TURN_ONLY_DEG:
            # WAYYYY OFFF: turn in place first (needs enough rotation to not stall)
            speed = 0
            if abs(rot) < cal.TURN_MIN_ROT:
                rot = cal.TURN_MIN_ROT if error > 0 else -cal.TURN_MIN_ROT

        else:
            # roughly facing the ball: drive forward, slower as we get close
            span = cal.APPROACH_SLOW_FT - cal.APPROACH_STOP_FT
            frac = (rs.ball_dist - cal.APPROACH_STOP_FT) / span # 0 at stop_ft, 1 at slow_ft
            frac = max(0.0, min(1.0, frac))
            speed = cal.APPROACH_MIN_SPEED + frac * (cal.APPROACH_SPEED - cal.APPROACH_MIN_SPEED)

        self.r.send_command(speed, 0, rot)
        self.r.drive_update()

    def PK(self):
        """
        Penalty Kick Mode
        Use one of the active switch states to put the robot into PK mode
        This will drive forward for a set amount of time to hit the ball and then stop
        """
        if time.time() - self.state_start_time > cal.PK_TIME:
            print("PK: done -> search")
            rs.pk_state = 0 # main loop stops calling PK
            self.set_state(0)
            return
        self.r.send_command(cal.PK_SPEED, 0, 0)
        self.r.drive_update()

    def rotate_for_kick(self):
        """
        state 2: get behind the ball.
        Orbit the ball (strafe sideways + rotate to keep facing it) until our heading
        matches rs.goal_heading, i.e. the ball is between us and their goal.
        Lined up -> attack(3). Lost ball -> search (0). Timeout -> attack anyway.
        """
        now = time.time()
        self.look_for_ball()

        # lost it?
        if now - rs.ball_seen_time > cal.APPROACH_LOST_TIME:
            print("get behind: lost ball -> search")
            self.set_state(0)
            return

        # taking too long (ball on wall, etc.)? DON'T BE SILLY. SEND IT!
        if now - self.state_start_time > cal.BEHIND_TIMEOUT:
            print("get behind: timeout. DON'T BE SILLY! SEND IT!!")
            self.set_state(3)
            return

        goal_err = wrap_180(rs.goal_heading - rs.angle_z) # + = need to swing CCW
        ball_err = wrap_180(rs.ball_bearing - rs.angle_z) # + = ball is left of center

        # lined up?
        if abs(goal_err) < cal.BEHIND_TOL and abs(ball_err) < cal.BEHIND_FACE_TOL:
            print(f"get behind: lined up (goal err {goal_err:.0f}) -> attack")
            self.r.send_command(0,0,0)
            self.r.drive_update(force=True)
            self.set_state(3)
            return

        # pick orbit direction: shorter way round, but near 180 keep the old
        # direction so it doesn't flip-flop when both ways are equally far
        if abs(goal_err) < 150:
            self.orbit_dir = 1 if goal_err > 0 else -1

        # sideways: slow down as we get close to lined up
        if abs(goal_err) < cal.BEHIND_TOL:
            strafe = 0 # heading is good, only fix facing
        else:
            strafe = clamp(cal.BEHIND_K_ORBIT * abs(goal_err), cal.BEHIND_MIN_STRAFE, cal.BEHIND_STRAFE_SPEED)

        left = -self.orbit_dir * strafe # CCW orbit = strafe RIGHT (negative left)

        # forward/back: hodl the orbit radius (only wiht a fresh distance)
        fwd = 0
        if rs.found_ball:
            fwd = clamp(cal.BEHIND_K_DIST * (rs.ball_dist - cal.BEHIND_RADIUS_FT), -cal.BEHIND_MAX_FWD, cal.BEHIND_MAX_FWD)

        # combine into one mecanum drive command
        speed = np.hypot(fwd, left)
        angle = np.degrees(np.arctan2(left,fwd))

        # rotation: feedforward from strafing + correction to keep ball centered
        rot = self.orbit_dir * cal. BEHIND_FF * strafe + cal.BEHIND_K_FACE * ball_err
        rot = clamp(rot, -cal.BEHIND_MAX_ROT, cal.BEHIND_MAX_ROT)
        if strafe == 0 and abs(fwd) < 30 and rot != 0:
            #turning nearly in place; make sure it's enough to not stall
            rot = rot / abs(rot) * max(abs(rot), cal.TURN_MIN_ROT)

        self.r.send_command(speed, angle, rot)
        self.r.drive_update()

    def goal_bearing(self):
        """
        Absolute bearing of the center of THEIR goal, if both posts are in the current frame and in the right order for rs.goal_state.  Otherwise None.
        Call after a process_frame that included "goal"
        """
        if not(rs.found_green_goal and rs.found_purple_goal):
            return None
        g = self.bearing_to(rs.green_x)
        pu = self.bearing_to(rs.purple_x)
        sep = wrap_180(pu - g)  # + = purple is left of green
        if (sep > 0) != bool(rs.goal_state):
            return None # posts in wrong order: that's our own goal dummy!
        return wrap_180(g + sep/2)

    def refine_goal_heading(self):
        """
        If their posts are in the current frame, nudge rs.goal_heading toward them.
        Returns the goal bearing seen this frame or None.
        Call after a process_frame that included "goal".
        """
        seen = self.goal_bearing()
        if seen is not None:
            rs.goal_heading = wrap_180(rs.goal_heading + cal.ATTACK_GOAL_BLEND * wrap_180(seen - rs.goal_heading))

        return seen

    def drive_into_goal(self):
        """
        state 3: attack. Push the ball at their goal.
        Rotation holds heading on the goal (gyro); strafing keeps the ball
        centered in front. Seeing their post refines rs.goal_heading.
        Lost ball -> search (0). Knocked off line -> approach (1)
        """
        now = time.time()
        self.look_for_ball(["goal"]) # one frame: ball + posts

        # refine goal heading from the real posts (blend so one bad frame can't yank it)
        self.refine_goal_heading()

        # lost ball? if it was last seen right in front, it's probably just under the camera
        if rs.ball_dist < cal.ATTACK_BLIND_FT:
            lost_limit = cal.ATTACK_BLIND_TIME
        else:
            lost_limit = cal.ATTACK_LOST_TIME
        if now - rs.ball_seen_time > lost_limit:
            print("attack: timeout -> search")
            self.set_state(0)
            return

        # been at it too long (scored, stuck on wall, ...)
        if now - self.state_start_time > cal.ATTACK_TIMEOUT:
            print("attack: tiemout -> search")
            self.set_state(0)
            return

        goal_err = wrap_180(rs.goal_heading - rs.angle_z) # + = goal is left
        ball_err = wrap_180(rs.ball_bearing - rs.angle_z) # + = ball is left

        # knocked off line? set up again
        if abs(goal_err) > cal.ATTACK_REALIGN_DEG or (rs.found_ball and abs(ball_err) > cal.ATTACK_REALIGN_DEG):
            print(f"attack: off line (goal {goal_err:.0f}, ball {ball_err:.0f}) -> approach")
            self.set_state(1)
            return
        # sideways: slide to keep the ball centered ( only on a fresh sighting)
        left = 0
        if rs.found_ball:
            left = clamp(cal.ATTACK_K_STRAFE * ball_err, -cal.ATTACK_MAX_STRAFE, cal.ATTACK_MAX_STRAFE)
        fwd = cal.ATTACK_SPEED
        speed = min(255, np.hypot(fwd, left))
        angle = np.degrees(np.arctan2(left, fwd))

        # rotation: hold heading on the goal
        rot = clamp(cal.ATTACK_K_HEAD * goal_err, -cal.ATTACK_MAX_ROT, cal.ATTACK_MAX_ROT)

        self.r.send_command(speed, angle, rot)
        self.r.drive_update()