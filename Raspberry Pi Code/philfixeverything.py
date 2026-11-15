"""
Tema Brute force mechatronics robot code that will win because we want it to

The computer vision is through opencv and detecting color ranges

The state machine states are outline as:
# States:
# -2: Defense mode. switch to 1 when we find the ball
# -1: initialize (find goal, set gyro)
# 0: Finding Ball (look around + back up till ball found then state 1)
# 1: Going To Ball (angle drive until distance is small then state 2) (if lost go to state 0)
# 2: Rotating for kick (center goal in frame with rotation then got to state 3)
# 3: Driving into goal (look up at goal and drive straight if other robot seen kick then go to state 0)
# 4: Zeroing angle (entered if IR sensor tripped from state 0 or 1)
# (rotate about triped wheel until 2 sensors trip then enter 0)


TODO
add comments NC
actually add scheduler
split CV into functions to reduce compute time for diff states (almost)
get purple and green thresh going (test in class)
distance estimate based on pixel distance of the goal posts 
work through method of how to tell robot what goal we want to score into (purple left or green left) switch
states fully defined for testing (make testing easy with locked in state and somefeedback on if it is working)

run ir all times to check if we gonna crash
plan how we do IR I think if we see, run away from sensor that tripped for a quarter second then back out to previous state

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
import gpiozero import AngularServo # used for Servo commands


fl_sensor = 23 # front left GPIO pin
bl_sensor = 27 # back left GPIO pin
fr_sensor = 17 # front right GPIO pin
br_sensor = 22 # Back right GPIO pin
goal_switch = 5 # high = purple left, low = green left
hat_switch = 6 # high = yellow hat low = blue hat
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

state = 0
# constants for colors
# stupid wrap around need 0-5 and 170 -180 ball
low_lower_red = np.array([0, 150, 100])
low_upper_red = np.array([5, 255, 255])
up_lower_red = np.array([170, 150, 100])
up_upper_red = np.array([180, 255, 255])
# hat
lower_yellow = np.array([25, 150, 100])
upper_yellow = np.array([40, 255, 255])
lower_orange = np.array([18, 90, 63])
upper_orange = np.array([30, 100, 90])
# goal corners
lower_green = np.array([95, 30, 55])
upper_green = np.array([115, 60, 100])
lower_purple = np.array([260, 50, 40])
upper_purple = np.array([280, 60, 75])
# need to add code for finding these and test if the range is good.

# orange data points
# [30.3 95.9 85.1]
# [18.8 99.4 69.4]
# [32.2 98.7 88.2]
# [21.8 99.4 69.4]
# [21.1 97.8 70.6]
# [21.2 99.4 64.7]
# [20.8 97 64.3]
# [19.9 100 66.3]

#Green data points
# [100, 55.2, 78.8]
# [112.7, 54.7, 58.8]
# [111.1, 52.6, 60.4]
# [97.2, 33.6, 92.2]
# [96.1, 36.4, 94.9]

# Purp data points
# [275.1, 53.1, 69.4]
# [276.8, 55.8, 71]
# [278.5, 52.9, 68.2]
# [264.8 , 55.3, 44.7]
# [269, 52.1, 46.7]
# [271.8, 56.4, 45.9]
# [278.7, 58.1, 53.3] 


def initialize():
    global ser, picam, GOAL_STATE, HAT_STATE
    ser = serial.Serial('/dev/ttyACM0', 115200)
    GPIO.setmode(GPIO.BCM)
    picam = Picamera2()
    picam.configure(picam.create_video_configuration(main={"format": 'XRGB8888', "size": (640, 480)})) # 640x480 picam 
    # init all IR sensors
    GPIO.setup(fl_sensor, GPIO.IN); GPIO.setup(bl_sensor, GPIO.IN); GPIO.setup(fr_sensor, GPIO.IN); GPIO.setup(br_sensor, GPIO.IN)
    # init goal switch and hat switch
    GPIO.setup(goal_switch, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    GPIO.setup(hat_switch, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    init_servos() # initialize camera servos
    GOAL_STATE = update_goal_state()
    HAT_STATE = update_hat_state()
    picam.start()
    time.sleep(2)
    # add switch read and set global variables

def update_goal_state():
    """
    update status of goal switch
    returns status of high = purple left, low = green left
    """
    GOAL_STATE = GPIO.input(goal_switch)
    if GOAL_STATE:
        return 1 # HIGH, PURPLE LEFT
    else:
        return 0 # LOW, GREEN LEFT

def update_hat_state():
    """
    update status of hat switch
    returns status of # high = yellow hat low = blue hat
    """
    HAT_STATE = GPIO.input(hat_switch)
    if HAT_STATE:
        return 1 # HIGH, YELLOW HAT
    else:
        return 0 # LOW, BLUE HAT



def IR_sensor_read(): # need to get working actually 
# Low = Black
        fl_val = 1 - GPIO.input(fl_sensor)
        fr_val = 1 - GPIO.input(fr_sensor)
        bl_val = 1 - GPIO.input(bl_sensor)
        br_val = 1 - GPIO.input(br_sensor)


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



# threads need to add
def camera_sensors_update(ball, hat, goals): # add arg for ball, hat, goals
    global state, angle_z, next_command
    while not shutting_down:
        # THIS IS THE OLD BLUE AND YELLOW CODE SO WE NEED NEW HAT COLORS AND GOAL IDENTIFICATION
        frame = picam.capture_array()
        frame = cv2.resize(frame, (320, 240))
        frameBlur = cv2.blur(frame, (9, 9))
        frameHSV = cv2.cvtColor(frameBlur, cv2.COLOR_BGR2HSV)

        # ball == True
        mask1 = cv2.inRange(frameHSV, low_lower_red, low_upper_red)
        mask2 = cv2.inRange(frameHSV, up_lower_red, up_upper_red)
        frameThreshRed = mask1 + mask2
        frameCannyRed = cv2.Canny(frameThreshRed, 150, 255, (7, 7))
        contoursRed, hierarchy = cv2.findContours(frameCannyRed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find the ball
        x_max_ball = 0
        y_max_ball = 0
        r_max = 20
        found_ball = False
        for i in contoursRed:
            ((x, y), r) = cv2.minEnclosingCircle(i)
            if r > r_max:
                found_ball = True
                x_max_ball = x
                y_max_ball = y
                r_max = r

        # Hats (need switch input for what color hat to look for or maybe run speed tests and do both)
        # read global variable for what hat color is enemy if we do this

        ###### add orange 
        frameThreshYellow = cv2.inRange(frameHSV, lower_yellow, upper_yellow)
        frameCannyYellow = cv2.Canny(frameThreshYellow, 150, 255, (7, 7))
        contoursYellow, hierarchy = cv2.findContours(frameCannyYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        x_max_yellow_hat = 0 # switch to just middle x and y 
        y_max_yellow_hat = 0
        w_max_yellow_hat = 0
        h_max_yellow_hat = 0
        found_yellow_hat = False
        A_max_yellow_hat = 400
        for i in contoursYellow:
            x, y, w, h = cv2.boundingRect(i)
            if w * h > A_max_yellow_hat:
                found_yellow_hat = True
                x_max_yellow_hat = x
                y_max_yellow_hat = y
                w_max_yellow_hat = w
                h_max_yellow_hat = h
                A_max_yellow_hat = w * h

        # Goal finding 
        frameThreshGreen = cv2.inRange(frameHSV, lower_green, upper_green)
        frameThreshPurple = cv2.inRange(frameHSV, lower_purple, upper_purple)      
        frameCannyGreen = cv2.Canny(frameThreshGreen, 150, 255, (7, 7))
        frameCannyPurple = cv2.Canny(frameThreshPurple, 150, 255, (7, 7))
        contoursGreen, hierarchy = cv2.findContours(frameCannyGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contourspurple, hierarchy = cv2.findContours(frameCannyPurple, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        x_max_green_goal = 0 
        y_max_green_goal = 0
        w_max_green_goal = 0
        h_max_green_goalt = 0
        found_green_goal = False
        A_max_green_goal = 400
        for i in contoursGreen:
            x, y, w, h = cv2.boundingRect(i)
            if w * h > A_max_green_goal:
                found_green_goal = True
                x_max_green_goal = x
                y_max_green_goal = y
        x_max_purple_goal = 0 
        y_max_purple_goal = 0
        w_max_purple_goal = 0
        h_max_purple_goalt = 0
        found_purple_goal = False
        A_max_purple_goal = 400
        for i in contoursPurple:
            x, y, w, h = cv2.boundingRect(i)
            if w * h > A_max_purple_goal:
                found_purple_goal = True
                x_max_purple_goal = x
                y_max_purple_goal = y


def state_machine():

    # Distances:
    # 2ft ~ r = 88px
    # 2.5 ft = 74 px
    # 3ft ~ r = 62px
    # 3.5 ~ r = 56 px
    # 4ft ~ r = 47px
    speed = 0
    rotation = 0
    angle = 0
    # HERE IS WHERE WE DO STATE MACHINE SHIT
    if state == -2:  # defense mode
        pass
    if state == -1:  # init
        if found_desire_goal and abs(x_desire_goal - 160) < 20:
            angle_z = 0
            state = 0
        else:
            speed = 0
            rotation = 100
            angle = 0
    if state == 0:
        if fl_val + fr_val + bl_val + br_val == 1:
            # If it is 2 we are facing forward
            state = 4
            break
        if found_ball:
            state = 1
            break
        # if backed up:
        #     state = -2
        # add look with servo
    # This is old code for chasing the ball prolly still good and for state 1
    if state == 1:
        if found_ball:
            predicted_distance = 10.307 - 0.0923 * r_max # distance in ft.
            hor_error = x_max_ball - 160  # hor_error is horizontal error from center screen pixel
            hor_pos = (7 / (2 * r_max)) * hor_error
            if predicted_distance > 2.1: # if the ball is greater than 2ft away we can determine angle to the ball
                angle = -np.arctan2(hor_pos, predicted_distance) * 180 / np.pi
            else:
                angle = 0 # if we're within 2 ft of the ball, it pretty much takes up the FOV, so set the angle to 0 and CHANGE STATE TO FIND THE GOAL with cam
            speed = 255
            frame = cv2.putText(frame, f"Hor error: {hor_error} cmd speed {speed}", (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball dist: {predicted_distance}ft", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball Size: {r_max}", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            frame = cv2.putText(frame, f"Ball dist: Not Detected", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball Size: Not Detected", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    next_command = f"<DRIVE|{int(speed)}, {int(angle)}, {int(rotation)}>"
    print(next_command)
    print(f"fl: {fl_val}, fr: {fr_val}, bl: {bl_val}, br: {br_val}")
    # state 3 is look up and angle to the goal and drive in etc etc
    # state 4 uses new noel code
    # TODO add a bunch of flags for which sections of CV we care about


def drive_update():
    while not shutting_down:
        ser.write(next_command.encode())
        time.sleep(1 / MOTOR_FREQ)


def imu_thingy_for_stuff():
    global angle_z
    prev_time = time.time()
    while not shutting_down:
        # Read sensor values
        _, _, gyro_z = mpu.gyro
        if abs(acc_y) < 0.2:
            acc_y = 0
        t_now = time.time()
        delta_t = t_now - prev_time
        prev_time = t_now
        angle_z += gyro_z * delta_t
        time.sleep(1 / IMU_FREQ)


# main:
def main():
    global shutting_down
    initialize()
    # TODO start the threads
    # TODo: prioritize and shit
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        shutting_down = True
        ser.write("<STOP>".encode())
        picam.close()
        cv2.destroyAllWindows()
        ser.close()
    # TODO: JOIN ALL ThrEADS