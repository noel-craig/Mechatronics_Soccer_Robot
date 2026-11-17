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
i dont see how this helps, we should check IR in all states, if any trigger drive away from the wall and return to that state to keep going.
the time wasted trying to reorient is wasted if the ball bounces off the wall and goes back toward the mid field
# (rotate about triped wheel until 2 sensors trip then enter 0)


TODO
add comments NC
actually add scheduler (maybe not and just limit speed for motor)
define states beyond pseudo code
get purple and green thresh going (test in class)
distance estimate based on pixel distance of the goal posts (add in state)

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
from gpiozero import AngularServo # used for Servo commands
import random


fl_sensor = 23 # front left GPIO pin
bl_sensor = 27 # back left GPIO pin
fr_sensor = 17 # front right GPIO pin
br_sensor = 22 # Back right GPIO pin
goal_switch = 5 # high = purple left, low = green left
hat_switch = 6 # high = yellow hat low = orange hat
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
last_motor_cmd_time = 0

# openCV global variables
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
min_goal_area = 200 # test if this works goals would be pretty small far away


heading = 0
GOAL_HEADING = 90 # set to due east so goal away from mountains. idk we can change 
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
    returns status of # high = yellow hat low = orange hat
    """
    HAT_STATE = GPIO.input(hat_switch)
    if HAT_STATE:
        return 1 # HIGH, YELLOW HAT
    else:
        return 0 # LOW, ORANGE HAT

def IR_sensor_read(): # need to get working actually 
# Low = Black
        fl_val = 1 - GPIO.input(fl_sensor)
        fr_val = 1 - GPIO.input(fr_sensor)
        bl_val = 1 - GPIO.input(bl_sensor)
        br_val = 1 - GPIO.input(br_sensor)

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
    
    frame = picam.capture_array()
    frame = cv2.resize(frame, (320, 240))
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # cases for ball, hat and goal using minenclosing circle to get centerpoints
    # call with predefine array of strings ["ball", "hat"]
    # case check for ball 
    if "ball" in targets: 
        mask1 = cv2.inRange(frameHSV, low_lower_red, low_upper_red)
        mask2 = cv2.inRange(frameHSV, up_lower_red, up_upper_red)
        frameThreshRed = mask1 + mask2
        contoursRed, hierarchy = cv2.findContours(frameThreshRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contoursRed:
            ball = max(contoursRed, key=cv2.contourArea) # only look at biggest contour
            if cv2.contourArea(ball) > min_ball_area: # above minimum size
                ((ball_x, ball_y), ball_r) = cv2.minEnclosingCircle(ball) # get coords
                found_ball = True
            else:
                found_ball = False
    else: 
        found_ball = False

    
    # case check for hat high yellow low orange
    if "hat" in targets:
        if HAT_STATE:
            # yellow hat on enemy
            frameThreshYellow = cv2.inRange(frameHSV, lower_yellow, upper_yellow)
            contoursYellow, hierarchy = cv2.findContours(frameThreshYellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contoursYellow:
                hat = max(contoursYellow, key=cv2.contourArea) #only biggest cluster found
                if cv2.contourArea(hat) > min_hat_area:
                    ((hat_x, hat_y), hat_r) = cv2.minEnclosingCircle(hat)
                    found_hat = True
                else:
                    found_hat = False
        else: 
            # orange hat on enemy
            frameThreshOrange = cv2.inRange(frameHSV, lower_orange, upper_orange)
            contoursOrange, hierarchy = cv2.findContours(frameThreshOrange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contoursOrange:           
                hat = max(contoursOrange, key=cv2.contourArea) #only biggest cluster found
                if cv2.contourArea(hat) > min_hat_area:
                    ((hat_x, hat_y), hat_r) = cv2.minEnclosingCircle(hat)
                    found_hat = True
                else:
                    found_hat = False   
    else:
        found_hat = False
        
    #case for goal blocks looking for both always and just output location
    if "goal" in targets:
        frameThreshGreen = cv2.inRange(frameHSV, lower_green, upper_green)
        frameThreshPurple = cv2.inRange(frameHSV, lower_purple, upper_purple)      
        contoursGreen, hierarchy = cv2.findContours(frameThreshGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursPurple, hierarchy = cv2.findContours(frameThreshPurple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contoursGreen:
            green = max(contoursGreen, key=cv2.contourArea) 
            if cv2.contourArea(green) > min_goal_area:
                ((green_x, green_y),green_r) = cv2.minEnclosingCircle(green)
                found_green_goal = True
            else:
                found_green_goal = False
        if contoursPurple:
            purple = max(contoursPurple, key=cv2.contourArea)    
            if cv2.contourArea(purple) > min_goal_area:
                ((purple_x, purple_y),purple_r) = cv2.minEnclosingCircle(purple)
                found_purple_goal = True
            else:
                found_purple_goal = False
    else:
        found_purple_goal = False
        found_green_goal = False
        
    """ for troubleshooting 
    frame = cv2.putText(frame, f"Hor error: {hor_error} cmd speed {speed}", (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    frame = cv2.putText(frame, f"Ball dist: {predicted_distance}ft", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    frame = cv2.putText(frame, f"Ball Size: {r_max}", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    frame = cv2.putText(frame, f"Ball dist: Not Detected", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    frame = cv2.putText(frame, f"Ball Size: Not Detected", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    """
                    
# threads need to add
# walk through to get rid of this
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
        contoursRed, hierarchy = cv2.findContours(frameThreshRed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # find the ball if ball == True
        
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

        # Hats (need switch input for what color hat to look for or maybe run speed tests and do both) if hat == True
        # read global variable for what hat color is enemy if we do this

        ###### add orange 
        frameThreshYellow = cv2.inRange(frameHSV, lower_yellow, upper_yellow)
        contoursYellow, hierarchy = cv2.findContours(frameThreshYellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
        contoursGreen, hierarchy = cv2.findContours(frameThreshGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursPurple, hierarchy = cv2.findContours(frameThreshPurple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        x_max_green_goal = 0 
        y_max_green_goal = 0
        w_max_green_goal = 0
        h_max_green_goal = 0
        found_green_goal = False 
        A_max_green_goal = 200 # play with size since these should be small
        for i in contoursGreen:
            x, y, w, h = cv2.boundingRect(i)
            if w * h > A_max_green_goal:
                found_green_goal = True
                x_max_green_goal = x
                y_max_green_goal = y
        x_max_purple_goal = 0 
        y_max_purple_goal = 0
        w_max_purple_goal = 0
        h_max_purple_goal = 0
        found_purple_goal = False
        A_max_purple_goal = 200 # play with size since these should be small
        for i in contoursPurple:
            x, y, w, h = cv2.boundingRect(i)
            if w * h > A_max_purple_goal:
                found_purple_goal = True
                x_max_purple_goal = x
                y_max_purple_goal = y
        
            
def state_machine(state):
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
    while state == -2:  # defense mode
        pass
    while state == -1:  # init
        print(state)
        process_frame(["goal"])
        if found_green_goal and found_purple_goal:
            print("found both goals")
            
            if GOAL_STATE: # purple left goal
                if green_x - purple_x > 0: # green is further right on camera than purple
                    heading = imu_thingy_for_stuff() # save heading when we see the goal we want
                    print(f"heading 4 goal {heading}")
                    GOAL_HEADING = heading
                    #state = 0
                else: # need to turn around we see the wrong goal
                    pass
                    #turn around faster
            else: # green left goal
                if purple_x - green_x > 0:
                    heading = imu_thingy_for_stuff() # save the heading 
                    print(f"heading 4 goal {heading}")
                    GOAL_HEADING = heading
                    #state = 0

        else:
            # didnt find goals keep spinning
            rand = random.randint(1,1000)
            speed = 0
            angle = 0
            if rand % 2 ==0:
                # rotate right
                rotation = 90
                send_command(speed, angle,rotation)
            else: 
                # rotate left
                rotation = -90
                send_command(speed,angle,rotation)
            drive_update()
        # need to write to motor to spin 
    while state == 0:
        """
        Finding Ball (look around + back up till ball found then state 1)
        
        check ir
        check camera for ball
        cam scan
        if found_ball true, change to state 1  
        if ball too damn close, state 2
        """
        """
        #if fl_val + fr_val + bl_val + br_val == 1:
            # If it is 2 we are facing forward
            state = 4
            break
        """
        IR_sensor_read()
        process_frame(["ball"])
        if found_ball:
            state = 1
            return
        # if backed up:
        #     state = -2
        # add look with servo
    # This is old code for chasing the ball prolly still good and for state 1
    while state == 1:
        """
        Going To Ball (angle drive until distance is small then state 2) (if lost go to state 0)
        
        check ir
        check camera update for ball location
        do drive to ball code,
        if found ball false, go to state 0 
        process_frame(["ball"])
        """
        
        if found_ball:
            predicted_distance = 10.307 - 0.0923 * ball_r # distance in ft.
            hor_error = ball_x - 160  # hor_error is horizontal error from center screen pixel
            hor_pos = (7 / (2 * ball_r)) * hor_error
            if predicted_distance > 2.1: # if the ball is greater than 2ft away we can determine angle to the ball
                angle = -np.arctan2(hor_pos, predicted_distance) * 180 / np.pi
            else:
                angle = 0 # if we're within 2 ft of the ball, it pretty much takes up the FOV, so set the angle to 0 and CHANGE STATE TO FIND THE GOAL with cam
            speed = 255
            
            """ for troubleshooting 
            frame = cv2.putText(frame, f"Hor error: {hor_error} cmd speed {speed}", (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball dist: {predicted_distance}ft", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball Size: {r_max}", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball dist: Not Detected", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball Size: Not Detected", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            """
    next_command = f"<DRIVE|{int(speed)}, {int(angle)}, {int(rotation)}>"
    print(next_command)
    
    while state ==2:
        """
        Rotating for kick (center goal in frame with rotation then got to state 3)
        
        check ir
        check camera for ball and goal pieces ()
            if one goal piece check with gyro for direction of our desired goal
        if found ball and both goal, drive to put goal intersection 
          
        """
        
    print(f"fl: {fl_val}, fr: {fr_val}, bl: {bl_val}, br: {br_val}")
    # state 3 is look up and angle to the goal and drive in etc etc
    # state 4 uses new noel code
    # TODO add a bunch of flags for which sections of CV we care about

def send_command(speed, angle, rotation):
    next_command = f"<DRIVE|{int(speed)}, {int(angle)}, {int(rotation)}>"



def drive_update():
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
    global angle_z
    prev_time = time.time()
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
    # hard code state for testing 
    state = -1
    state_machine(state)
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