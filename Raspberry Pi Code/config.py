import random
import board
import adafruit_mpu6050
from gpiozero import AngularServo

# GLOBALS READ-ONLY

fl_sensor = 23 # front left GPIO pin
fr_sensor = 27 # back left GPIO pin
bl_sensor = 17 # front right GPIO pin
br_sensor = 22 # Back right GPIO pin
goal_switch = 5 # high = purple left, low = green left
pk_switch = 6 # high = yellow hat low = orange hat
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