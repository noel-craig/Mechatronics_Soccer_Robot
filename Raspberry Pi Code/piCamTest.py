import time
from picamera2 import Picamera2, Preview
import cv2
import numpy as np
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

picam = Picamera2()

picam.configure(picam.create_video_configuration(main={"format":'XRGB8888',"size":(640, 480)}))
#config = picam.create_preview_configuration()
#picam.configure(config)

#picam.start_preview(Preview.QTGL)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
picam.start()

while True:
    frame = picam.capture_array()
    frame = cv2.resize(frame,(320,240))
    frameBlur = cv2.blur(frame,(9,9))
    frameHSV = cv2.cvtColor(frameBlur, cv2.COLOR_BGR2HSV)
    
    low_lower_red = np.array([0,150,100])
    low_upper_red = np.array([5,255,255])
    up_lower_red = np.array([170,150,100])
    up_upper_red = np.array([180,255,255])
    
    lower_blue = np.array([100,150,100])
    upper_blue = np.array([130,255,255])

    lower_yellow = np.array([25,150,100])
    upper_yellow = np.array([40,255,255])
    
    mask1=cv2.inRange(frameHSV,low_lower_red,low_upper_red)
    mask2=cv2.inRange(frameHSV,up_lower_red,up_upper_red)
    
    frameThreshRed = mask1+mask2
    frameThreshBlue = cv2.inRange(frameHSV,lower_blue,upper_blue)
    frameThreshYellow = cv2.inRange(frameHSV,lower_yellow,upper_yellow)
    
    frameCannyRed = cv2.Canny(frameThreshRed,150,255, (7,7))
    frameCannyBlue = cv2.Canny(frameThreshBlue,150,255, (7,7))
    frameCannyYellow = cv2.Canny(frameThreshYellow,150,255, (7,7))
    contoursRed, hierarchy = cv2.findContours(frameCannyRed,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contoursBlue, hierarchy = cv2.findContours(frameCannyBlue,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contoursYellow, hierarchy = cv2.findContours(frameCannyYellow,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    x_max_ball = 0
    y_max_ball = 0
    r_max = 20
    found_ball = False
    for i in contoursRed:
        ((x,y),r) = cv2.minEnclosingCircle(i)
        if r > r_max:
            found_ball = True
            x_max_ball = x
            y_max_ball = y
            r_max = r
            
        
    x_max_blue_hat = 0
    y_max_blue_hat = 0
    w_max_blue_hat = 0
    h_max_blue_hat = 0
    found_blue_hat = False
    A_max_blue_hat = 400
    for i in contoursBlue:
        x,y,w,h = cv2.boundingRect(i)
        if w*h > A_max_blue_hat:
            found_blue_hat = True
            x_max_blue_hat = x
            y_max_blue_hat = y
            w_max_blue_hat = w
            h_max_blue_hat = h
            A_max_blue_hat = w*h
            
            
    x_max_yellow_hat = 0
    y_max_yellow_hat = 0
    w_max_yellow_hat = 0
    h_max_yellow_hat = 0
    found_yellow_hat = False
    A_max_yellow_hat = 400
    for i in contoursYellow:
        x,y,w,h = cv2.boundingRect(i)
        if w*h > A_max_blue_hat:
            found_yellow_hat = True
            x_max_yellow_hat = x
            y_max_yellow_hat = y
            w_max_yellow_hat = w
            h_max_yellow_hat = h
            A_max_yellow_hat = w*h
            
            
    if found_ball :
        cv2.circle(frame,(int(x_max_ball), int(y_max_ball)), int(r_max), (0,0,255), 5)
        GPIO.output(17,GPIO.HIGH)
    else:
        GPIO.output(17, GPIO.LOW)
        
    if found_blue_hat :
        cv2.rectangle(frame,(x_max_blue_hat, y_max_blue_hat),(x_max_blue_hat+w_max_blue_hat, y_max_blue_hat+h_max_blue_hat), (255,0,0), 5)
        GPIO.output(27,GPIO.HIGH)
    else:
        GPIO.output(27, GPIO.LOW)

    if found_yellow_hat :
        cv2.rectangle(frame,(x_max_yellow_hat, y_max_yellow_hat),(x_max_yellow_hat+w_max_yellow_hat, y_max_yellow_hat+h_max_yellow_hat), (0,255,255), 5)
        GPIO.output(22,GPIO.HIGH)
    else:
        GPIO.output(22, GPIO.LOW)

    frame = cv2.resize(frame,(640,480))
    
#     cv2.imshow('FRAMEBLUR', frameBlur)
    cv2.imshow('Framediff', frameThreshBlue)
    cv2.imshow('Frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    


picam.close()

cv2.destroyAllWindows()


