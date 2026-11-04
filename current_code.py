import time
from picamera2 import Picamera2, Preview
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial

ser = serial.Serial('/dev/ttyACM0',115200)
time.sleep(1)

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

try:
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
                
        #Distances:
        #2ft ~ r = 88px
        #2.5 ft = 74 px
        #3ft ~ r = 62px
        #3.5 ~ r = 56 px
        #4ft ~ r = 47px
        speed = 0
        rotation = 0
        angle = 0
        if(found_ball):
            #predicted_distance = 9.35-0.154*r_max
            predicted_distance = 10.307-0.0923*r_max
            hor_error = x_max_ball - 160 # hor_error is horizontal error from center screen pixel
            hor_pos = (7/(2*r_max) * hor_error)
            if(predicted_distance > 2.1):
                angle = - np.arctan2(hor_pos,predicted_distance)*180/np.pi
            else:
                angle = 0
            
            speed = 255
            if speed > 255:
                speed = 255

            
            # drive and shit
            # drive right so 270 direction 
#             if hor_error > 5:
#                 angle = 270
#                 speed = hor_error*2 + 50
#             elif hor_error < -5:
#                 angle = 90
#                 speed = hor_error*-2 + 50
#             if speed > 255:
#                 speed = 255

#             command = f"<DRIVE|{speed}, {angle}, {rotation}>"
#             print(command)
#             ser.write(command.encode())
# #             
            frame = cv2.putText(frame, f"Hor error: {hor_error} cmd speed {speed}", (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball dist: {predicted_distance}ft", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball Size: {r_max}", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
#             command = f"<STOP>"
#             print(command)
#             ser.write(command.encode())
            frame = cv2.putText(frame, f"Ball dist: Not Detected", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            frame = cv2.putText(frame, f"Ball Size: Not Detected", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        command = f"<DRIVE|{int(speed)}, {int(angle)}, {int(rotation)}>"
        print(command)
#         ser.write(command.encode())
#             
            
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
                
                
#         if found_ball :
#             cv2.circle(frame,(int(x_max_ball), int(y_max_ball)), int(r_max), (0,0,255), 5)
#              GPIO.output(17,GPIO.HIGH)
#         else:
#              GPIO.output(17, GPIO.LOW)
#             
#         if found_blue_hat :
#             cv2.rectangle(frame,(x_max_blue_hat, y_max_blue_hat),(x_max_blue_hat+w_max_blue_hat, y_max_blue_hat+h_max_blue_hat), (255,0,0), 5)
#              GPIO.output(27,GPIO.HIGH)
#         else:
#             GPIO.output(27, GPIO.LOW)
# 
#         if found_yellow_hat :
#             cv2.rectangle(frame,(x_max_yellow_hat, y_max_yellow_hat),(x_max_yellow_hat+w_max_yellow_hat, y_max_yellow_hat+h_max_yellow_hat), (0,255,255), 5)
#              GPIO.output(22,GPIO.HIGH)
#         else:
#              GPIO.output(22, GPIO.LOW)

        #frame = cv2.resize(frame,(640,480))
        
    #     cv2.imshow('FRAMEBLUR', frameBlur)
        #cv2.imshow('Framediff', frameThreshBlue)
        cv2.imshow('Frame', frame)
        ser.read_all()
        time.sleep(0.25)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        
except KeyboardInterrupt:
    ser.write("<STOP>".encode())
    picam.close()
    cv2.destroyAllWindows()
    ser.close()


