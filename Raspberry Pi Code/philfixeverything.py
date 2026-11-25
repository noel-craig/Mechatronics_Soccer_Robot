"""
Team Brute Force mechatronics robot code that will win because we want it to

The computer vision is through opencv and detecting color ranges

TODO
actually add scheduler? (maybe not and just limit speed for motor)
distance estimate based on pixel distance of the goal posts (add in state)

to test:
    orange
    imu 90 degree good enough for goal check
    speed for ball chasing
    defense mode
    do we need state 0 of back up to find ball, defense state does that?
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

shutting_down = False
next_command = "<STOP>"
should_kick = False
last_motor_cmd_time = time.time_ns()

# ---- debug display ----
# set False for competition (imshow slows the loop and needs a monitor/VNC/ssh -X)
DEBUG_VIEW = True
SHOW_MASKS = False # also show the black/white threshold masks for each color (great for tuning HSV)

state = State_Machine(sys.modules[__name__]) # pass this module in so states.py can call its functions (process_frame, send_command, ...)
prev_time = time.time()
gyro_bias = 0

def initialize():
    global ser, picam
    ser = serial.Serial('/dev/ttyACM0', 115200)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(10, GPIO.OUT)
    GPIO.output(10, GPIO.HIGH)
    picam = Picamera2()
    picam.configure(picam.create_video_configuration(main={"format": 'XRGB8888', "size": (640, 480)}, # still stays 640x480
                                                     raw={"size": (1640 ,1232)})) # but gets full FOV
    # init all IR sensors
    GPIO.setup(fl_sensor, GPIO.IN); GPIO.setup(bl_sensor, GPIO.IN); GPIO.setup(fr_sensor, GPIO.IN); GPIO.setup(br_sensor, GPIO.IN)
    # init goal switch and hat switch
    GPIO.setup(goal_switch, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(pk_switch, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    init_servos() # initialize camera servos
    rs.goal_state = update_goal_state()
    rs.pk_state = update_PK_state()
    picam.start()
    calibrate_gyro()
    # time.sleep(2) THIS MOVED INSIDE CALIBRATE GYRO
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
        rs.ir_fl = 1 - GPIO.input(fl_sensor)
        rs.ir_fr = 1 - GPIO.input(fr_sensor)
        rs.ir_bl = 1 - GPIO.input(bl_sensor)
        rs.ir_br = 1 - GPIO.input(br_sensor)

#================== SERVO =======================================
def init_servos():
    """
    Set both servos to 90 degrees
    """
    rs.cam_pan = 90
    hor_servo.angle = rs.cam_pan
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
    rs.cam_pan = hor_servo.angle

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
            rs.cam_pan = hor_servo.angle
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
                rs.ball_area = cv2.contourArea(ball)
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
        IR_sensor_read()

        if state.check_escape():
            continue
        
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
            state.rotate_for_kick()
        elif state.number == 3:
            state.drive_into_goal()
        else:
            state.init()

def send_command(speed, angle, rotation):
    global next_command
    # Python side: + angle = left, + rotation = CCW. Arduino is opposite, so both are negated here.
    next_command = f"<DRIVE|{int(speed)}, {int(-angle)}, {int(-rotation)}>"

def drive_update(force = False):
    global last_motor_cmd_time
    curr_time = time.time_ns()
    if force or curr_time - last_motor_cmd_time > 1e8:
        ser.write(next_command.encode())
        last_motor_cmd_time = curr_time
    else: 
        time.sleep(1/30)
        # time.sleep(1 / MOTOR_FREQ)
        # not sleep but add timing and check last time write and make sure enough time
        # passed between current and the last motor write 

def go(angle, speed=100, t=0.3):
    """
    USED IN DEBUGGING IN THE TERMINAL
    """
    send_command(speed, angle, 0)
    ser.write(p.next_command.encode())
    time.sleep(t)
    ser.write(b"<STOP>")

def imu_thingy_for_stuff():
    global prev_time
    # Read sensor values
    _, _, gyro_z = mpu.gyro
    gyro_z = np.degrees(gyro_z) - gyro_bias
    if abs(gyro_z) < 0.8:
        gyro_z = 0
    t_now = time.time()
    delta_t = t_now - prev_time
    prev_time = t_now
    rs.angle_z += gyro_z * delta_t
    return rs.angle_z

def calibrate_gyro():
    """
    Calibrates the gyro at the beginning of the init to generate the bias.
    This allows us to have more fine-tuning in the degrees recorded by the gyro.
    """
    global gyro_bias, prev_time
    print("hold still, calibrating gyro...")
    readings = []
    start = time.time()
    # read as many readings over 2 seconds
    while time.time() - start < 2.0:
        _,_,gyro_z = mpu.gyro
        gyro_z = np.degrees(gyro_z)
        readings.append(gyro_z)
        time.sleep(0.01)
    gyro_bias = sum(readings) / len(readings) # average of readings
    if max(readings) - min(readings) > 5:
        print("WARNING: GYRO BIAS > 5 degrees. Robot Moved.")
    print(f"GYRO BIAS: {gyro_bias:.3f}")
    prev_time = time.time()
    rs.angle_z = 0


# main:
def main():
    global shutting_down, state
    initialize()
    state.state_start_time = time.time()
    state.phase_start = time.time()
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