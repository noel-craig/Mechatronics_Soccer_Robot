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
imu 90 degree good enough for goal check
speed for ball chasing
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
import sys
from states import State_Machine
from config import * # hardware: pins, servos, imu
from calibration import * # tuning numbers from calibration.json
from robot_state import rs # shared live state (found_ball, ball_x, angle_z, ...)


# fl_sensor = 23 # front left GPIO pin
# fr_sensor = 27 # back left GPIO pin
# bl_sensor = 17 # front right GPIO pin
# br_sensor = 22 # Back right GPIO pin
# goal_switch = 5 # high = purple left, low = green left
# hat_switch = 6 # high = yellow hat low = orange hat
# switch_high = 10
# hor_servo = AngularServo(12, min_angle = 0, max_angle = 180, min_pulse_width = 0.0005, max_pulse_width = 0.0025) # horizonal rotation servo
# vert_servo = AngularServo(13, min_angle = 0, max_angle = 180, min_pulse_width = 0.0005, max_pulse_width = 0.0025) # vertical servo
# SERVO_SLEEP = 0.01 # time between angles for servos
# ser = None
# picam = None
# i2c = board.I2C()  # uses board.SCL and board.SDAs
# adafruit_mpu6050._MPU6050_DEVICE_ID = 0x98
# mpu = adafruit_mpu6050.MPU6050(i2c)
# SENSOR_FREQ = 30  # hz
# MOTOR_FREQ = 10  # hz
# IMU_FREQ = 60  # hz
# vision results, heading and switch states live in robot_state.py (rs.found_ball, rs.angle_z, ...)
# tuning numbers (HSV ranges, min areas, defense values) live in calibration.json
shutting_down = False
next_command = "<STOP>"
should_kick = False
last_motor_cmd_time = time.time_ns()

# ---- debug display ----
# set False for competition (imshow slows the loop and needs a monitor/VNC/ssh -X)
DEBUG_VIEW = True
SHOW_MASKS = True # also show the black/white threshold masks for each color (great for tuning HSV)


state = State_Machine(sys.modules[__name__]) # pass this module in so states.py can call its functions (process_frame, send_command, ...)
prev_time = time.time()


def initialize():
    global ser, picam
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
    GPIO.setup(pk_switch, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    init_servos() # initialize camera servos
    rs.goal_state = update_goal_state()
    rs.pk_state = update_PK_state()
    picam.start()
    time.sleep(2)
    print("init complete")

def update_goal_state():
    """
    update status of goal switch
    returns status of high = purple left, low = green left
    """
    goal_state = GPIO.input(goal_switch)
    if goal_state:
        print("goal Switch is high this way/purple left")
        return 1 # HIGH, PURPLE LEFT
    else:
        print("goal switch is low/green left")
        return 0 # LOW, GREEN LEFT

def update_PK_state():
    """
    update status of hat switch
    returns status of # high = yellow hat low = orange hat
    """
    pk_state = GPIO.input(pk_switch)
    if pk_state:
        print("PENALTY KICKK!!!!!")
        return 1
    else:
        return 0

def IR_sensor_read(): # need to get working actually 
# Low = Black
        fl_val = 1 - GPIO.input(fl_sensor)
        fr_val = 1 - GPIO.input(fr_sensor)
        bl_val = 1 - GPIO.input(bl_sensor)
        br_val = 1 - GPIO.input(br_sensor)

        #print(f"fl: {fl_val}, fr: {fr_val}, bl: {bl_val}, br: {br_val}")

        if (fl_val or fr_val or bl_val or br_val):
            speed = -100
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
            command = f"<DRIVE|{int(rotation)}, {int(angle)}, {int(-speed)}>"
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
    # results go into rs (robot_state.py) so states.py sees them
    # reset if it saw something last loop
    rs.reset_vision()

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
                ((rs.ball_x, rs.ball_y), rs.ball_r) = cv2.minEnclosingCircle(ball) # get coords
                rs.found_ball = True

    
    # case check for hat high yellow low orange
    if "hat" in targets:
        if rs.hat_state:
            # yellow hat on enemy
            frameThreshYellow = cv2.inRange(frameHSV, lower_yellow, upper_yellow)
            masks["yellow (hat)"] = frameThreshYellow
            contoursYellow, hierarchy = cv2.findContours(frameThreshYellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contoursYellow:
                hat = max(contoursYellow, key=cv2.contourArea) #only biggest cluster found
                draw_blob(debug, hat, "hat", min_hat_area, (0, 255, 255))
                if cv2.contourArea(hat) > min_hat_area:
                    ((rs.hat_x, rs.hat_y), hat_r) = cv2.minEnclosingCircle(hat)
                    rs.found_hat = True
        else:
            # orange hat on enemy
            frameThreshOrange = cv2.inRange(frameHSV, lower_orange, upper_orange)
            masks["orange (hat)"] = frameThreshOrange
            contoursOrange, hierarchy = cv2.findContours(frameThreshOrange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contoursOrange:
                hat = max(contoursOrange, key=cv2.contourArea) #only biggest cluster found
                draw_blob(debug, hat, "hat", min_hat_area, (0, 140, 255))
                if cv2.contourArea(hat) > min_hat_area:
                    ((rs.hat_x, rs.hat_y), hat_r) = cv2.minEnclosingCircle(hat)
                    rs.found_hat = True
        
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
                ((rs.green_x, rs.green_y), green_r) = cv2.minEnclosingCircle(green)
                rs.found_green_goal = True
        if contoursPurple:
            purple = max(contoursPurple, key=cv2.contourArea)
            draw_blob(debug, purple, "purple", min_goal_area, (255, 0, 200))
            if cv2.contourArea(purple) > min_goal_area:
                ((rs.purple_x, rs.purple_y), purple_r) = cv2.minEnclosingCircle(purple)
                rs.found_purple_goal = True

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
    cv2.putText(img, f"state {state.number}  hdg {rs.angle_z:.0f}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    flags = f"B:{int(rs.found_ball)} H:{int(rs.found_hat)} G:{int(rs.found_green_goal)} P:{int(rs.found_purple_goal)}"
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
    print(f"spin dir is {state.spin_dir}")

    while not shutting_down:
        # run imu for gyro heading (updates rs.angle_z)
        imu_thingy_for_stuff()
        if rs.pk_state:
            state.PK()
        elif state.number == -2:  # defense mode where the hell da ball
            state.defense()
        elif state.number == -1:  # init
            state.init()
        elif state.number == 0:
            state.find_ball()
        elif state.number == 1:
            state.go_to_ball()
        elif state.number == 2:
            state.state_2()
        elif state.number == 3:
            state.state_3()
        else:
            state.init()
        # state 4 uses new noel code
        # TODO add a bunch of flags for which sections of CV we care about

def send_command(speed, angle, rotation):
    global next_command
    next_command = f"<DRIVE|{int(rotation)}, {int(angle)}, {int(-speed)}>"



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
    global prev_time
    # Read sensor values
    _, _, gyro_z = mpu.gyro
    gyro_z = np.degrees(gyro_z)
    if abs(gyro_z) < 2:
        gyro_z = 0
    t_now = time.time()
    delta_t = t_now - prev_time
    prev_time = t_now
    rs.angle_z += gyro_z * delta_t
    return rs.angle_z


# main:
def main():
    global shutting_down, state
    initialize()
    
    # TODO start the threads
    # TODo: prioritize and shit
    # hard code state for testing 
    state.number = -1
    
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