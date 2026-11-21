from philfixeverything import *
from config import *

class State_Machine():
    def __init__(self, state_number = -1):
        self.number = state_number
        self.spin_dir = random.choice([-90, 90]) # random left or right direction might still be too fast
        self.speed = 0
        self.angle = 0
        self.rotation = 0
        self.angle_z = 0

    def defense(self):

        """
        Defense: face the goal we score on (with +/-90 deg of GOAL_HEADING),
            back-up toward our own goal until the opposing goal posts look close enough together,
            then wait until we see the ball.
        """
        IR_sensor_read()
        process_frame(["ball", "goal"])
        imu_thingy_for_stuff()
        # how far off we are from GOAL_HEADING, wrapped to -180..180
        heading_error = (GOAL_HEADING - angle_z + 180) % 360 - 180

        if found_ball:
            print("defense: found ball")
            self.number = 1
        elif abs(heading_error) > DEFENSE_HEADING_TOL:
            # facing the wrong way, rotate toward GOAL_HEADING before backing up
            print(f"defense: wrong way, heading err {heading_error:.1f}, rotating")
            send_command(0, 0, 90 if heading_error > 0 else -90) # check rotation sign for direction
            drive_update()
        elif found_green_goal and found_purple_goal:
            goal_px = abs(green_x - purple_x) # gets smaller the further we are from the goal
            print(f"defense: goal px sep {goal_px:.0f} (target {DEFENSE_GOAL_PX}), heading err {heading_error:.1f}")
            if goal_px > DEFENSE_GOAL_PX:
                # still too close to their goal, keep backing up
                send_command(150, 180, 0) # 180 = straight back, check angle
            else:
                # at our end of the field, stop and wait for the ball
                print("defense: in position")
                send_command(0, 0, 0)
                # can go into wiggle
                drive_update()
        else:
            # facing roughly right but cant see both posts, rotate toward GOAL_HEADING to find them
            print(f"defense: goals not both seen, heading err {heading_error:.1f}")
            send_command(0, 0, 60 if heading_error > 0 else -60) # check rotation sign
            drive_update()

    def init(self):
        """
        Initialize State:
        """
        #print(f"the state is {state}")
        self.number = -1
        IR_sensor_read()
        process_frame(["goal"])
        if found_green_goal and found_purple_goal:
            print("found both goals")
            goal_px = abs(green_x - purple_x) # gets smaller the further we are from the goal
            print(f"defense: goal px sep {goal_px:.0f} (target {DEFENSE_GOAL_PX})")
        
            if GOAL_STATE: # purple left goal
                if green_x - purple_x > 0: # green is further right on camera than purple
                    heading = imu_thingy_for_stuff() # save heading when we see the goal we want
                    print(f"heading 4 goal {heading}")
                    GOAL_HEADING = heading
                    send_command(0,0,0)
                    ser.write("<STOP>".encode())
                    print("Going to state 0")
                    self.number = 0
                else: # need to turn around we see the wrong goal
                    send_command(0,0,self.spin_dir)
                    drive_update()
                    #turn around faster maybe
            else: # green left goal
                if purple_x - green_x > 0:
                    heading = imu_thingy_for_stuff() # save the heading 
                    print(f"heading 4 goal {heading}")
                    GOAL_HEADING = heading
                    send_command(0,0,0)
                    ser.write("<STOP>".encode())
                    self.number = 0
                else: 
                    send_command(0,0,self.spin_dir)
                    # troubleshoot try to drive fwd
                    # send command(100,0,0) it did not drive right
                    drive_update()
        else:
            # didnt find goals keep spinning
            print("no goals, spinning")
            #tilt_cam(135)
            #time.sleep(1)
            #tilt_cam(90)
            #time.sleep(1)
            send_command(0, 0, self.spin_dir)
            drive_update()

    def find_ball(self):
        """
        Finding Ball (look around + back up till ball found then state 1)

        check ir
        check camera for ball
        cam scan
        if found_ball true, change to state 1  
        if ball too damn close, state 2
        """
        IR_sensor_read()
        process_frame(["ball", "goal"])
        if found_ball:
            self.number = 1
        # backup and cam scan
        send_command(-100, 0, 0)
        drive_update()
        cam_sweep(0,45,135)

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
        IR_sensor_read()
        process_frame(["ball"])

        if found_ball:
            predicted_distance = 10.307 - 0.0923 * ball_r # distance in ft.
            hor_error = ball_x - 160  # hor_error is horizontal error from center screen pixel
            hor_pos = (7 / (2 * ball_r)) * hor_error
            if predicted_distance > 2.1: # if the ball is greater than 2ft away we can determine angle to the ball
                self.angle = -np.arctan2(hor_pos, predicted_distance) * 180 / np.pi
            else:
                self.angle = 0 # if we're within 2 ft of the ball, it pretty much takes up the FOV, so set the angle to 0 and CHANGE STATE TO FIND THE GOAL with cam
            self.speed = 120
            send_command(self.speed, self.angle, self.rotation)
            drive_update()
            # print(f"dist {predicted_distance:.2f}ft  angle {angle:.1f}  cmd {next_command}")
            # if predicted_distance <= 2.1: # close enough, line up the kick
            #     send_command(0, 0, 0)
            #     drive_update()
            #     state = 2

        else:
            send_command(0,0,0)
            drive_update()
            print("lost ball")
            self.number = 0 # go back to find the ball now that we lost it

    def PK(self):
        """
        Penalty Kick Mode
        Use one of the active switch states to put the robot into PK mode
        This will drive forward for a set amount of time to hit the ball and then stop
        """
        send_command(200, 0, 0)