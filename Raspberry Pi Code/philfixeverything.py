"""
Tema Brute force mechatronics robot code that will win because we want it to

The computer vision is through opencv and detecting color ranges

to start you need to get into the vitrual environment

source .venv/bin/activate

The state machine states are outline as:
# States:
# -2: Defense mode. switch to 1 when we find the ball
# -1: initialize (find goal, set gyro)
# 0: Finding Ball (look around + back up till ball found then state 1)
# 1: Going To Ball (angle drive until distance is small then state 2) (if lost go to state 0)
# 2: Rotating for kick (center goal in frame with rotation then got to state 3)
# 3: Driving into goal (look up at goal and drive straight if other robot seen kick then go to state 0)
# 4: Zeroing angle (entered if IR sensor tripped from state 0 or 1) 
i dont see how this helps, we should check IR in all states, if any trigger drive away from the wall and return to that state to keep going.
the time wasted trying to reorient is wasted if the ball bounces off the wall and goes back toward the mid field
# (rotate about triped wheel until 2 sensors trip then enter 0)


TODO
add comments NC
actually add scheduler (maybe not and just limit speed for motor)
get purple and green thresh going (test in class)
distance estimate based on pixel distance of the goal posts (add in state)

do we want to remove state 0 of find ball because defense should be what we do anyway? 


states fully defined for testing (make testing easy with locked in state and somefeedback on if it is working)
run ir all times to check if we gonna crash

to test:
orange
purple
green
pixel minimum size for goals sepearation from one side of the field to other
motor speed for backup
find goal init state
imu working and if 90 degree good enough 
speed for ball chasing
rotation speed
IR working, driving away right, 0.25 sec long enough to get off wall
defense mode
do we need state 0 of back up to find ball, defense state does that.
scoring state logic






"""

import threading
import time
import board
import adafruit_mpu6050
from picamera2 import Picamera2, Preview
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial
from gpiozero import AngularServo # used for Servo commands
import random


fl_sensor = 23 # front left GPIO pin
bl_sensor = 27 # back left GPIO pin
fr_sensor = 17 # front right GPIO pin
br_sensor = 22 # Back right GPIO pin
goal_switch = 5 # high = purple left, low = green left
hat_switch = 6 # high = yellow hat low = orange hat
switch_high = 10
hor_servo = AngularServo(12, min_angle = 0, max_angle = 180, min_pulse_width = 0.0005, max_pulse_width = 0.0025) # horizonal rotation servo
vert_servo = AngularServo(13, min_angle = 0, max_angle = 180, min_pulse_width = 0.0005, max_pulse_width = 0.0025) # vertical servo
SERVO_SLEEP = 0.01 # time between angles for servos
ser = None
picam = None
i2c = board.I2C()  # uses board.SCL and board.SDAs
adafruit_mpu6050._MPU6050_DEVICE_ID = 0x98
mpu = adafruit_mpu6050.MPU6050(i2c)
SENSOR_FREQ = 30  # hz
MOTOR_FREQ = 10  # hz
IMU_FREQ = 60  # hz
angle_z = 0
shutting_down = False
next_command = "<STOP>"
should_kick = False
last_motor_cmd_time = time.time_ns()

# openCV global variables
frame = 0
found_ball = False
ball_x = 0
ball_y = 0
ball_r = 0
found_hat = False
hat_x = 0
hat_y = 0
found_green_goal = False
green_x = 0
green_y = 0
found_purple_goal = False
purple_x = 0
purple_y = 0
min_ball_area = 1250
min_hat_area = 400
min_goal_area = 50 # test if this works goals would be pretty small far away
DEFENSE_GOAL_PX = 40 # CALIBRATE: goal post x separation (px) when we are back at our end of the field
DEFENSE_HEADING_TOL = 90 # must be within this many degrees of GOAL_HEADING to back up

# ---- debug display ----
# set False for competition (imshow slows the loop and needs a monitor/VNC/ssh -X)
DEBUG_VIEW = True
SHOW_MASKS = True # also show the black/white threshold masks for each color (great for tuning HSV)


heading = 0
GOAL_HEADING = 90 # set to due east so goal away from mountains. idk we can change 
state = -1
prev_time = time.time()

# constants for colors
# stupid wrap around need 0-5 and 170 -180 ball
low_lower_red = np.array([0, 150, 100])
low_upper_red = np.array([5, 255, 255])
up_lower_red = np.array([170, 150, 100])
up_upper_red = np.array([180, 255, 255])
# hat
lower_yellow = np.array([25, 150, 100])
upper_yellow = np.array([40, 255, 255])
# old values were color picker scale (H 0-360, S 0-100, V 0-100)
# converted to opencv scale (H 0-179, S 0-255, V 0-255): H/2, S*2.55, V*2.55
# lower rounded down, upper rounded up
# lower_orange = np.array([18, 90, 63])
# upper_orange = np.array([30, 100, 90])
lower_orange = np.array([9, 229, 160])
upper_orange = np.array([15, 255, 230])
# goal corners
# lower_green = np.array([95, 30, 55])
# upper_green = np.array([115, 60, 100])
# lower_purple = np.array([260, 50, 40])
# upper_purple = np.array([280, 60, 75])
lower_green = np.array([47, 76, 140])
upper_green = np.array([58, 153, 255])
lower_purple = np.array([130, 127, 102])
upper_purple = np.array([140, 153, 192])
# need to add code for finding these and test if the range is good.


def initialize():
    global ser, picam, GOAL_STATE, HAT_STATE
    ser = serial.Serial('/dev/ttyACM0', 115200)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(10, GPIO.OUT)
    GPIO.output(10, GPIO.HIGH)
    picam = Picamera2()
    picam.configure(picam.create_video_configuration(main={"format": 'XRGB8888', "size": (640, 480)})) # 640x480 picam 
    # init all IR sensors
    GPIO.setup(fl_sensor, GPIO.IN); GPIO.setup(bl_sensor, GPIO.IN); GPIO.setup(fr_sensor, GPIO.IN); GPIO.setup(br_sensor, GPIO.IN)
    # init goal switch and hat switch
    GPIO.setup(goal_switch, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(hat_switch, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    init_servos() # initialize camera servos
    GOAL_STATE = update_goal_state()
    HAT_STATE = update_hat_state()
    picam.start()
    time.sleep(2)
    print("init complete")
    # add switch read and set global variables

def update_goal_state():
    """
    update status of goal switch
    returns status of high = purple left, low = green left
    """
    GOAL_STATE = GPIO.input(goal_switch)
    if GOAL_STATE:
        print("goal Switch is high this way/purple left")
        return 1 # HIGH, PURPLE LEFT
    else:
        print("goal switch is low/green left")
        return 0 # LOW, GREEN LEFT

def update_hat_state():
    """
    update status of hat switch
    returns status of # high = yellow hat low = orange hat
    """
    HAT_STATE = GPIO.input(hat_switch)
    if HAT_STATE:
        print("hat switch high/yellow")
        return 1 # HIGH, YELLOW HAT
    else:
        print("hat switch low/orange")
        return 0 # LOW, ORANGE HAT

def IR_sensor_read(): # need to get working actually 
# Low = Black
        fl_val = 1 - GPIO.input(fl_sensor)
        fr_val = 1 - GPIO.input(fr_sensor)
        bl_val = 1 - GPIO.input(bl_sensor)
        br_val = 1 - GPIO.input(br_sensor)

        print(f"fl: {fl_val}, fr: {fr_val}, bl: {bl_val}, br: {br_val}")

        if (fl_val or fr_val or bl_val or br_val):
            speed = 120
            rotation = 0
            if(fl_val):
                angle = -135
            elif(fr_val):
                angle = 135
            elif(bl_val):
                angle = -45
            elif(br_val):
                angle = 45

            # write drive away command
            command = f"<DRIVE|{int(speed)}, {int(angle)}, {int(rotation)}>"
            ser.write(command.encode())
            # wait 1/4 sec
            time.sleep(0.25) # check if this is long enough
            # stop it 
            ser.write("<STOP>".encode())

#================== SERVO =======================================
def init_servos():
    """
    Set both servos to 90 degrees
    """
    hor_servo.angle = 90
    vert_servo.angle = 90

def tilt_cam(angle):
    """
    tilt the camera up or down to a specific angle
    """
    vert_servo.angle = angle

def rotate_cam(angle):
    """
    rotate the camera left and right to a specific angle
    """
    hor_servo.angle = angle

def cam_sweep(rotation, min_angle, max_angle):
    """
    sweep the camera vertically or horizontally
    input
        rotation(int): 0 = hor_servo, 1 = vert_servo
        min_angle(int): starting angle
        max_angle(int): final angle
    """
    if rotation == 0:
        for i in range(max_angle - min_angle):
            hor_servo.angle = min_angle + i 
            time.sleep(SERVO_SLEEP)
    elif rotation == 1:
        for i in range(max_angle - min_angle):
            vert_servo.angle = min_angle + i
            time.sleep(SERVO_SLEEP)
    else:
        init_servos()
#================================================================

# just going to find things and leave computation to states
def process_frame(targets):
    global found_ball, ball_r, ball_x, ball_y
    global found_hat, hat_x, hat_y
    global found_green_goal, green_x, green_y
    global found_purple_goal, purple_x, purple_y
    # reset if it saw something last loop
    found_ball = found_hat = found_green_goal = found_purple_goal = False
    
    frame = picam.capture_array()
    framesmall = cv2.resize(frame, (320, 240))
    frameHSV = cv2.cvtColor(framesmall, cv2.COLOR_BGR2HSV)
    
    # debug copy to draw on. drawn on the small frame so the x/y coords match what the states use
    # XRGB8888 gives 4 channels, drop the 4th so drawing colors work normally
    debug = framesmall[:, :, :3].copy() if DEBUG_VIEW else None
    masks = {} # name -> threshold mask, shown if SHOW_MASKS

    # call with predefine array of strings ["ball", "hat"]
    # case check for ball 
    if "ball" in targets: 
        mask1 = cv2.inRange(frameHSV, low_lower_red, low_upper_red)
        mask2 = cv2.inRange(frameHSV, up_lower_red, up_upper_red)
        frameThreshRed = mask1 + mask2
        masks["red (ball)"] = frameThreshRed
        contoursRed, hierarchy = cv2.findContours(frameThreshRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contoursRed:
            ball = max(contoursRed, key=cv2.contourArea) # only look at biggest contour
            draw_blob(debug, ball, "ball", min_ball_area, (0, 0, 255))
            if cv2.contourArea(ball) > min_ball_area: # above minimum size
                ((ball_x, ball_y), ball_r) = cv2.minEnclosingCircle(ball) # get coords
                found_ball = True

    
    # case check for hat high yellow low orange
    if "hat" in targets:
        if HAT_STATE:
            # yellow hat on enemy
            frameThreshYellow = cv2.inRange(frameHSV, lower_yellow, upper_yellow)
            masks["yellow (hat)"] = frameThreshYellow
            contoursYellow, hierarchy = cv2.findContours(frameThreshYellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contoursYellow:
                hat = max(contoursYellow, key=cv2.contourArea) #only biggest cluster found
                draw_blob(debug, hat, "hat", min_hat_area, (0, 255, 255))
                if cv2.contourArea(hat) > min_hat_area:
                    ((hat_x, hat_y), hat_r) = cv2.minEnclosingCircle(hat)
                    found_hat = True
        else:
            # orange hat on enemy
            frameThreshOrange = cv2.inRange(frameHSV, lower_orange, upper_orange)
            masks["orange (hat)"] = frameThreshOrange
            contoursOrange, hierarchy = cv2.findContours(frameThreshOrange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contoursOrange:
                hat = max(contoursOrange, key=cv2.contourArea) #only biggest cluster found
                draw_blob(debug, hat, "hat", min_hat_area, (0, 140, 255))
                if cv2.contourArea(hat) > min_hat_area:
                    ((hat_x, hat_y), hat_r) = cv2.minEnclosingCircle(hat)
                    found_hat = True
        
    #case for goal blocks looking for both always and just output location
    if "goal" in targets:
        frameThreshGreen = cv2.inRange(frameHSV, lower_green, upper_green)
        frameThreshPurple = cv2.inRange(frameHSV, lower_purple, upper_purple)
        masks["green (goal)"] = frameThreshGreen
        masks["purple (goal)"] = frameThreshPurple
        contoursGreen, hierarchy = cv2.findContours(frameThreshGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursPurple, hierarchy = cv2.findContours(frameThreshPurple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contoursGreen:
            green = max(contoursGreen, key=cv2.contourArea)
            draw_blob(debug, green, "green", min_goal_area, (0, 255, 0))
            if cv2.contourArea(green) > min_goal_area:
                ((green_x, green_y),green_r) = cv2.minEnclosingCircle(green)
                found_green_goal = True
        if contoursPurple:
            purple = max(contoursPurple, key=cv2.contourArea)
            draw_blob(debug, purple, "purple", min_goal_area, (255, 0, 200))
            if cv2.contourArea(purple) > min_goal_area:
                ((purple_x, purple_y),purple_r) = cv2.minEnclosingCircle(purple)
                found_purple_goal = True

    show_debug(debug, masks)

def draw_blob(img, contour, name, min_area, color):
    """
    draw the biggest blob of a color on the debug image.
    solid circle + label = big enough to count as found
    thin outline + "too small" = color matched but under the min area (lower min area or widen HSV)
    """
    if img is None:
        return
    area = cv2.contourArea(contour)
    (x, y), r = cv2.minEnclosingCircle(contour)
    x, y, r = int(x), int(y), int(r)
    if area > min_area:
        cv2.drawContours(img, [contour], -1, color, 1)
        cv2.circle(img, (x, y), r, color, 2)
        cv2.circle(img, (x, y), 2, color, -1) # center dot
        cv2.putText(img, f"{name} x{x} a{area:.0f}", (x - r, max(y - r - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    else:
        cv2.drawContours(img, [contour], -1, color, 1)
        cv2.putText(img, f"{name} too small a{area:.0f}<{min_area}", (x, max(y - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

def show_debug(img, masks):
    """
    show the debug window. cv2.imshow does NOTHING on screen until cv2.waitKey runs,
    that is why the old imshow never showed up
    """
    if img is None:
        return
    # state + found flags across the top
    cv2.putText(img, f"state {state}  hdg {angle_z:.0f}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    flags = f"B:{int(found_ball)} H:{int(found_hat)} G:{int(found_green_goal)} P:{int(found_purple_goal)}"
    cv2.putText(img, flags, (5, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.line(img, (160, 0), (160, 240), (255, 255, 255), 1) # center line for lining up
    cv2.imshow("robot view", cv2.resize(img, (640, 480))) # blow up so it is easier to see
    if SHOW_MASKS:
        for name, m in masks.items():
            cv2.imshow(f"mask {name}", m)
    cv2.waitKey(1) # REQUIRED for the windows to actually draw

                    
def state_machine():
    # Distances:
    # 2ft ~ r = 88px
    # 2.5 ft = 74 px
    # 3ft ~ r = 62px
    # 3.5 ~ r = 56 px
    # 4ft ~ r = 47px
    global state, GOAL_HEADING

    # pick spin direction once so it doesnt flip every frame
    spin_dir = random.choice([-75, 75]) # random left or right direction might still be too fast
    print(f"spin dir is {spin_dir}")

    while not shutting_down:
        speed = 0
        rotation = 0
        angle = 0
        # HERE IS WHERE WE DO STATE MACHINE SHIT
        # run imu for gyro heading
        angle_z = imu_thingy_for_stuff()

        if state == -2:  # defense mode where the hell da ball
            """
            Defense: face the goal we score on (within 90 deg of GOAL_HEADING),
            back up toward our own end until the goal posts look close together,
            then wait. switch to 1 when ball found
            """
            IR_sensor_read()
            process_frame(["ball", "goal"])
            imu_thingy_for_stuff()
            # how far off we are from GOAL_HEADING, wrapped to -180..180
            heading_error = (GOAL_HEADING - angle_z + 180) % 360 - 180

            if found_ball:
                print("defense: found ball")
                state = 1
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
        elif state == -1:  # init
            #print(f"the state is {state}")
            process_frame(["goal"])
            if found_green_goal and found_purple_goal:
                print("found both goals")
            
                if GOAL_STATE: # purple left goal
                    if green_x - purple_x > 0: # green is further right on camera than purple
                        heading = imu_thingy_for_stuff() # save heading when we see the goal we want
                        print(f"heading 4 goal {heading}")
                        GOAL_HEADING = heading
                        send_command(0,0,0)
                        ser.write("<STOP>".encode())
                        print("Going to state 0")
                        #state = 0
                    else: # need to turn around we see the wrong goal
                        send_command(0,0,spin_dir)
                        drive_update()
                        #turn around faster
                else: # green left goal
                    if purple_x - green_x > 0:
                        heading = imu_thingy_for_stuff() # save the heading 
                        print(f"heading 4 goal {heading}")
                        GOAL_HEADING = heading
                        send_command(0,0,0)
                        ser.write("<STOP>".encode())
                        #state = 0
                    else: 
                        send_command(0,0,spin_dir)
                        # troubleshoot try to drive fwd
                        # send command(100,0,0) it did not drive right
                        drive_update()
            else:
                # didnt find goals keep spinning
                print("no goals, spinning")
                tilt_cam(135)
                time.sleep(1)
                tilt_cam(90)
                time.sleep(1)
                send_command(0, 0, spin_dir)
                drive_update()
        elif state == 0:
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
                state = 1
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
        elif state == 1:
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
                    angle = -np.arctan2(hor_pos, predicted_distance) * 180 / np.pi
                else:
                    angle = 0 # if we're within 2 ft of the ball, it pretty much takes up the FOV, so set the angle to 0 and CHANGE STATE TO FIND THE GOAL with cam
                speed = 255
                # send_command(speed, angle, rotation)
                # drive_update()
                # print(f"dist {predicted_distance:.2f}ft  angle {angle:.1f}  cmd {next_command}")
                # if predicted_distance <= 2.1: # close enough, line up the kick
                #     send_command(0, 0, 0)
                #     drive_update()
                #     state = 2

            else:
                send_command(0,0,0)
                drive_update()
                print("lost ball")
                state = 0 # go back to find the ball now that we lost it

    
        elif state ==2:
            """
            Rotating for kick (center goal in frame with rotation then got to state 3)
        
            check ir
            check camera for ball and goal pieces ()
                if one goal piece check with gyro for direction of our desired goal
            if found ball and both goal, drive to put goal intersection 
          
            """
            IR_sensor_read()
            process_frame(["ball","goal"])

            # if not found_ball:
            #     print("lost ball while lining up")
            #     state = 0
            # elif found_green_goal and found_purple_goal:
            #     goal_center_x = (green_x + purple_x) / 2
            #     goal_error = goal_center_x - ball_x  # want goal center lined up behind the ball
            #     ball_error = ball_x - 160            # keep ball centered in frame
            #     print(f"goal err {goal_error:.0f}  ball err {ball_error:.0f}")
            #     if abs(goal_error) < 15 and abs(ball_error) < 20:
            #         send_command(0, 0, 0)
            #         drive_update()
            #         state = 3
            #     else:
            #         # orbit the ball: strafe sideways while rotating to keep ball centered
            #         strafe_angle = 90 if goal_error < 0 else -90   # check sign, negative angle = right
            #         rotation = -0.5 * ball_error                   # check sign
            #         send_command(120, strafe_angle, rotation)
            #         drive_update()
            # else:
            #     # cant see both goal posts, use gyro to turn toward GOAL_HEADING
            #     heading_error = (GOAL_HEADING - angle_z + 180) % 360 - 180
            #     print(f"no goal in view, heading err {heading_error:.1f}")
            #     strafe_angle = 90 if heading_error > 0 else -90    # check sign
            #     send_command(120, strafe_angle, -0.5 * (ball_x - 160))
            #     drive_update()

        # elif state == 3:
        #     """
        #     Driving into goal (drive straight at goal, kick if enemy hat seen, then state 0)
        #     """
        #     IR_sensor_read()
        #     tilt_cam(110) # look up at goal, check angle
        #     process_frame(["ball", "goal", "hat"])
        #     if found_green_goal and found_purple_goal:
        #         goal_center_x = (green_x + purple_x) / 2
        #         rotation = -0.5 * (goal_center_x - 160) # steer toward goal center, check sign
        #         send_command(255, 0, rotation)
        #     else:
        #         # hold gyro heading
        #         heading_error = (GOAL_HEADING - angle_z + 180) % 360 - 180
        #         send_command(255, 0, 2 * heading_error) # check sign
        #     drive_update()
        #     print(f"driving to goal cmd {next_command}  hat {found_hat}")
        #     if found_hat:
        #         print("enemy seen, kick")
        #         # ser.write("<KICK>".encode()) # arduino does not have a KICK command yet
        #         tilt_cam(90)
        #         state = 0
        #     # if not found_ball: (ball may be hidden under camera when tilted up)
        #     #     tilt_cam(90)
        #     #     state = 0

        # state 4 uses new noel code
        # TODO add a bunch of flags for which sections of CV we care about

def send_command(speed, angle, rotation):
    global next_command
    next_command = f"<DRIVE|{int(speed)}, {int(angle)}, {int(rotation)}>"



def drive_update():
    global last_motor_cmd_time
    curr_time = time.time_ns()
    if curr_time - last_motor_cmd_time > 1e8:
        ser.write(next_command.encode())
        last_motor_cmd_time = curr_time

    else: 
        time.sleep(1/30)
    # time.sleep(1 / MOTOR_FREQ)
    # not sleep but add timing and check last time write and make sure enough time
    # passed between current and the last motor write 

def imu_thingy_for_stuff():
    global angle_z, prev_time
    # Read sensor values
    _, _, gyro_z = mpu.gyro
    gyro_z = np.degrees(gyro_z)
    if abs(gyro_z) < 0.5:
        gyro_z = 0
    t_now = time.time()
    delta_t = t_now - prev_time
    prev_time = t_now
    angle_z += gyro_z * delta_t
    return angle_z


# main:
def main():
    global shutting_down, state
    initialize()
    
    # TODO start the threads
    # TODo: prioritize and shit
    # hard code state for testing 
    state = -1
    
    try:
        state_machine()
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("fine we will stop, damn calm down")
    finally:
        shutting_down = True
        ser.write("<STOP>".encode())
        picam.close()
        cv2.destroyAllWindows()
        ser.close()
        GPIO.cleanup()
    # TODO: JOIN ALL ThrEADS

if __name__ == "__main__": main()