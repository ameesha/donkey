'''
vehicles.py

Class to pull together all parts that operate the vehicle including,
sensors, actuators, pilots and remotes.
'''

import cv2
import numpy as np
from random import uniform
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

from donkey.actuators import PWMSteeringActuator
from donkey.pilots import KerasCategorical
from donkey.remotes import RemoteClient
from donkey.sensors import PiVideoStream

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True

rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)


class BaseVehicle:
    def __init__(self,
                 drive_loop_delay: float=0.5,
                 camera: PiVideoStream=None,
                 actuator_mixer: PWMSteeringActuator=None,
                 pilot: KerasCategorical=None,
                 remote: RemoteClient=None):

        self.drive_loop_delay = drive_loop_delay #how long to wait between loops

        # these need to be updated when vehicle is defined
        self.camera = camera
        self.actuator_mixer = actuator_mixer
        self.pilot = pilot
        self.remote = remote

    def start(self):
        start_time = time.time()
        angle = 0.
        throttle = 0.

        # capture frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array

            blur = cv2.blur(image, (5,5))

            #hsv to complicate things, or stick with BGR
            #hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
            #thresh = cv2.inRange(hsv,np.array((0, 200, 200)), np.array((20, 255, 255)))

            #lower = np.array([76,31,4],dtype="uint8")
            #upper = np.array([225,88,50], dtype="uint8")
            
            #pink
            lower = np.array([100, 30, 160],dtype="uint8")
            upper = np.array([165, 88, 225], dtype="uint8")

            thresh = cv2.inRange(blur, lower, upper)
            thresh2 = thresh.copy()

            # find contours in the threshold image
            image, contours, hierarchy = cv2.findContours(
                thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # finding contour with maximum area and store it as best_cnt
            max_area = 0
            best_cnt = 1
            for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > max_area:
                            max_area = area
                            best_cnt = cnt

            # finding centroids of best_cnt and draw a circle there
            M = cv2.moments(best_cnt)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cv2.circle(blur,(cx,cy),10,(0,0,255),-1)

            # Note: using blob here
            angle = ((640 - cx) / 640 * 2) - 1

            # throttle
            # breaks faster (even tho it can't go backwards)
            throttle = -1
            if max_area > 0:
                (x, y), radius = cv2.minEnclosingCircle(best_cnt)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(thresh2, center, radius, (0, 0, 255), 2)

                # Note: Using circle here
                # If it's too small don't follow
                # If it's too big don't follow
                # Otherwise map to between 0.2 and 0.4 throttle
                min_radius = 30
                max_radius = 80
                min_throttle = 0.3
                max_throttle = 0.4
                if radius > min_radius and radius < max_radius:
                    throttle = max_throttle - ((max_throttle - min_throttle) * ((radius - min_radius) / (max_radius - min_radius)))

            self.actuator_mixer.update(throttle, angle)
            print('\n CAR: cx: {}, cy: {}, max_area: {}, angle: {:+04.2f}, throttle: {:+04.2f}'.format(
                cx, cy, max_area, angle, throttle), end='')

            cv2.imshow("Frame", blur)
            cv2.imshow('thresh', thresh2)
            # show the frame
            key = cv2.waitKey(1) & 0xFF
    
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

        
        # drive loop
        # while True:
            # now = time.time()
            # start = now

            # milliseconds = int( (now - start_time) * 1000)

            # get image array image from camera (threaded)
            # img_arr = self.camera.capture_arr()

            # angle, throttle, drive_mode = self.remote.decide_threaded(img_arr, angle, throttle, milliseconds)

            # if drive_mode == 'local':
            #     angle, throttle = self.pilot.decide(img_arr)

            # elif drive_mode == 'local_angle':
            #     # only update angle from local pilot
            #     angle, _ = self.pilot.decide(img_arr)

            # do some open cv stuff here
            # blur = cv2.blur(img_arr, (5, 5))
            # pink
            # lower = np.array([160, 30, 100], dtype="uint8")
            # upper = np.array([225, 88, 165], dtype="uint8")

            # thresh = cv2.inRange(blur, lower, upper)

            # cv2.imshow("Frame", blur)
            # cv2.imshow("thresh", thresh)
            # key = cv2.waitKey(1) & 0xFF

            # angle = 0
            # throttle = 0
            # angle = uniform(-1, 1)
            # throttle = uniform(-1, 1)
            # print('\n len(img_arr)={}, len(img_arr[0])={}'.format(len(img_arr), len(img_arr[0])))

            # self.actuator_mixer.update(throttle, angle)

            # print current car state
            # end = time.time()
            # lag = end - start
            # print('\n CAR: angle: {:+04.2f}   throttle: {:+04.2f}   drive_mode: {}  lag: {:+04.2f}'.format(
                # angle, throttle, drive_mode, lag), end='')
            
            # time.sleep(self.drive_loop_delay)
