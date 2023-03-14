# For mouse control
from pynput.mouse import Button, Controller

# For camera module
from picamera import PiCamera
from picamera.array import PiRGBArray

# For servo control
import RPi.GPIO as GPIO

# For image processing
import numpy as np
import cv2

import time
import subprocess

# initializing Picamera
camera = PiCamera()
camera.framerate = 90
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size = (640, 480))

# setting up pin 12 for servo as PWM
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(12, GPIO.OUT)
#servo = GPIO.PWM(12, 50)
#servo.start(0)

# Init mouse
mouse = Controller()

# Define parameters for the required blob
params = cv2.SimpleBlobDetector_Params()

# setting the thresholds
params.minThreshold = 150
params.maxThreshold = 250

# filter by color
params.filterByColor = 1
params.blobColor = 255

# filter by circularity
params.filterByCircularity = 1
params.minCircularity = 0.68

# filter by area
params.filterByArea = 1
params.minArea = 30
# params.maxArea = 1500

# creating object for SimpleBlobDetector
detector = cv2.SimpleBlobDetector_create(params)


flag = 0
cnt = 0
points = []
lower_blue = np.array([255, 255, 0])
upper_blue = np.array([255, 255, 0])

# Function for Pre-processing
def last_frame(img):
    cv2.imwrite("/home/pi/Desktop/lastframe1.jpg", img)
    img = cv2.GaussianBlur(img, (5, 5), 0)
    cv2.imwrite("/home/pi/Desktop/lastframe2.jpg", img)
    retval, img = cv2.threshold(img, 64, 255, cv2.THRESH_BINARY)
    cv2.imwrite("/home/pi/Desktop/lastframe3.jpg", img)
    img = cv2.resize(img, (28, 28), interpolation=cv2.INTER_AREA)
    cv2.imwrite("/home/pi/Desktop/lastframe4.jpg", img)
    img = cv2.dilate(img, (3, 3))
    cv2.imwrite("/home/pi/Desktop/lastframe.jpg", img)
    #output = subprocess.check_output(['python3', '/home/pi/Desktop/HarryPotterWandsklearn.py'])
    print(output[1])
    if output[1] == "0":
        print("Alohamora!!")
        #servo.ChangeDutyCycle(6.5)
        time.sleep(1.5)
        print("Box Opened!!")
    if output[1] == "2":
        print("Close!!")
        #servo.ChangeDutyCycle(3.5)
        print("Box Closed!!")
        time.sleep(1.5)

time.sleep(0.1)

for image in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
    frame = image.array
    frame = cv2.resize(frame, (frame.shape[1]//2, frame.shape[0]//2))
    frame =cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    

    #detecting keypoints
    keypoints = detector.detect(frame)
    #frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    frame_with_keypoints = frame.copy()
    for marker in keypoints:
        frame_with_keypoints = cv2.drawMarker(frame_with_keypoints, tuple(int(i) for i in marker.pt), color=(0, 255, 0))

    #starting and ending circle
    #frame_with_keypoints = cv2.circle(frame_with_keypoints, (140, 70), 6, (0, 255, 0), 2)
    #frame_with_keypoints = cv2.circle(frame_with_keypoints, (190, 140), 6, (0, 0, 255), 2)


    #points_array = cv2.KeyPoint_convert(keypoints)
    points_array = cv2.KeyPoint_convert(keypoints)

    if flag == 1:
        time.sleep(0.001)
        flag = 0

    if len(points_array) != 0:
        if flag == 0:
            if int(points_array[0][0]) in range(0, 255) and int(points_array[0][1]) in range(0, 255):
                time.sleep(0.5)
                #print("Start Tracing!!")
                print("Detected!!")
                print(cnt)
                cnt += 1
                mouse.press(Button.left)
                mouse.release(Button.left)
                flag = 1    

                
    #cv2.imshow("video",frame_with_keypoints)
    #cv2.imshow("video 2",frame)
    rawCapture.truncate(0)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break


cv2.destroyAllWindows()
#servo.stop()
#GPIO.cleanup()
