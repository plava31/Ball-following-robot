import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
 
GPIO.setmode(GPIO.BOARD)
 
GPIO.setwarnings(False)
 
TRIG = 16
ECHO = 18
 
MOTOR1B=29
MOTOR1E=31
 
MOTOR2B=21
MOTOR2E=19
 
 
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
 
GPIO.setup(MOTOR1B,GPIO.OUT)
 
GPIO.setup(MOTOR1E,GPIO.OUT)
 
GPIO.setup(MOTOR2B,GPIO.OUT)
GPIO.setup(MOTOR2E,GPIO.OUT)
 
GPIO.setwarnings(False)
 
def sonar(TRIG, ECHO):
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(2)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG,GPIO.LOW)
 
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
 
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
 
 
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance,0)
    return distance
    
def reverse():
    GPIO.output(MOTOR1B, GPIO.HIGH)
    GPIO.output(MOTOR1E, GPIO.LOW)
    GPIO.output(MOTOR2B, GPIO.HIGH)
    GPIO.output(MOTOR2E, GPIO.LOW)
 
def forward():
    GPIO.output(MOTOR1B, GPIO.LOW)
    GPIO.output(MOTOR1E, GPIO.HIGH)
    GPIO.output(MOTOR2B, GPIO.LOW)
    GPIO.output(MOTOR2E, GPIO.HIGH)
 
def left():
    GPIO.output(MOTOR1B, GPIO.HIGH)
    GPIO.output(MOTOR1E, GPIO.LOW)
    GPIO.output(MOTOR2B, GPIO.LOW)
    GPIO.output(MOTOR2E, GPIO.LOW)
 
def right():
    GPIO.output(MOTOR1B, GPIO.LOW)
    GPIO.output(MOTOR1E, GPIO.HIGH)
    GPIO.output(MOTOR2B, GPIO.LOW)
    GPIO.output(MOTOR2E, GPIO.LOW)
 
def stop():
    GPIO.output(MOTOR1B, GPIO.LOW)
    GPIO.output(MOTOR1E, GPIO.LOW)
    GPIO.output(MOTOR2B, GPIO.LOW)
    GPIO.output(MOTOR2E, GPIO.LOW)
 
def segment_color(frame):
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
 
    lower_green= np.array([36, 79, 74])
    upper_green= np.array([59, 255, 255])
    mask=cv2.inRange(hsv, lower_green, upper_green)
    largest_contour=0
    cont_index=0
    _ ,contours, _ =cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for idx,contour in enumerate(contours):
        area= cv2.contourArea(contour)
        if (area> largest_contour):
            largest_contour=area
            cont_index=idx
    r=(0,0,2,2)
    if len(contours)>0:
        r=cv2.boundingRect(contours[cont_index])
       
    return r, largest_contour
 
camera=PiCamera()
camera.resolution=(160,120)
 
rawCapture=PiRGBArray(camera,size=(160,120))
camera.framerate=16
time.sleep(0.1)
 
for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    image=frame.array
    cv2.imshow("cam",image)
    global center_x
    global center_y
    center_x=0
    center_y=0
    
    loct,area=segment_color(image)
    print("Area", area)
    x,y,w,h=loct
    dist=sonar(TRIG,ECHO)
    print(dist)
    if (w*h)<10:
        found=0
    else:
        found=1
        simg2=cv2.rectangle (image,(x,y),(x+w,y+h),255,2)
        center_x=x+((w)/2)
        center_y=y+((h)/2)
        cv2.circle(image,(int(center_x), int(center_y)),3,(0,110,255),-1)
        center_x-=80
        print("x", center_x)
    initial=700
    flag=0
    if (found==1):
        if(area<initial): 
            if(dist>40):
                forward()
                time.sleep(1.5)
                stop()
                time.sleep(2)
        if (area>initial):
            if(dist>15):
                if (center_x<=20 or center_x>=20):
                    if (-20<center_x<0):
                        print("taking a slight right")
                        right()
                        time.sleep(0.5)
                    elif (-80<center_x<-20):
                        print("taking a right turn")
                        right()
                        time.sleep(1.5)
                    elif (0<center_x<20):
                        print("taking a slight left")
                        left()
                        time.sleep(1)
                    elif(20<center_x<80):
                        print("taking a left turn")
                        left()
                        time.sleep(2)
                forward()
                time.sleep(2)
                stop()
                time.sleep(2)
            elif(dist<15):
                print("Reaching Green Object")
                stop()
                time.sleep(2)
 
    rawCapture.truncate(0)
 
    key = cv2.waitKey(1) & 0xFF    
 
    if key == ord('q'):
        break
    
GPIO.cleanup()
