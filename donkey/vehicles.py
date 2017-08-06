'''
vehicles.py

Class to pull together all parts that operate the vehicle including,
sensors, actuators, pilots and remotes.
'''

import cv2
import numpy as np
import time

from picamera import PiCamera
from picamera.array import PiRGBArray
from random import uniform

import donkey as dk

from donkey.actuators import PWMSteeringActuator
from donkey.pilots import KerasCategorical
from donkey.remotes import RemoteClient
from donkey.sensors import PiVideoStream
from threading import Thread
from constants import Constants


class CameraStream:
    def __init__(self):
        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.camera.resolution = (Constants.res_length, Constants.res_width)
        self.camera.framerate = Constants.frame_rate
        self.camera.hflip = True

        self.rawCapture = PiRGBArray(camera, size=(Constants.res_length, Constants.res_width))

        # allow the camera to warmup
        time.sleep(0.1)
    
    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # capture frames from the camera
        start_time = time.time()
        count = 0
        for frame in self.camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            count += 1
            now = time.time()
            elapsed_ms = int((now - start_time) * 1000)
            print('\n got frame, count={}, elapsed_ms={}, fps={}', count, elapsed_ms, 1000 * count/elapsed_ms)
            rawCapture.truncate(0)
            self.frame = frame.array
    
    def read(self):
        return self.frame


class BaseVehicle:
    def __init__(self, actuator_mixer: PWMSteeringActuator=None):
        # these need to be updated when vehicle is defined
        self.actuator_mixer = actuator_mixer
        self.camera = CameraStream()

    def start(self):
        # self.capture_frame()
        self.camera.start()

    # def capture_frame(self):
    #     # capture frames from the camera
    #     start_time = time.time()
    #     count = 0
    #     for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #         count += 1
    #         now = time.time()
    #         elapsed_ms = int((now - start_time) * 1000)

    #         print('\n got frame, count={}, elapsed_ms={}, fps={}', count, elapsed_ms, 1000 * count/elapsed_ms)
    #         # image = frame.array
    #         # t = Thread(self.calculate_throttle_and_angle(image), args=())
    #         # t.daemon = True
    #         # t.start()

    #         # clear the stream in preparation for the next frame
    #         rawCapture.truncate(0)

    def calculate_throttle_and_angle(self, image):
        blur = cv2.blur(image, (Constants.blur_anchor_x, Constants.blur_anchor_y))
            
        # pink
        lower_light = np.array([40, 5, 85],dtype="uint8")
        upper_light = np.array([70, 35, 115], dtype="uint8")
        thresh_light = cv2.inRange(blur, lower_light, upper_light)

        lower_dark = np.array([0, 0, 35],dtype="uint8")
        upper_dark = np.array([10, 10, 70], dtype="uint8")
        thresh_dark = cv2.inRange(blur, lower_dark, upper_dark)

        lower_mid = np.array([10, 0, 60],dtype="uint8")
        upper_mid = np.array([60, 30, 90], dtype="uint8")
        thresh_mid = cv2.inRange(blur, lower_mid, upper_mid)

        thresh = thresh_light + thresh_dark + thresh_mid
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
        angle = ((Constants.angle_blob - cx) / Constants.angle_blob * 2) - 1

        # throttle
        # breaks faster (even tho it can't go backwards)
        throttle = 0
        if max_area > 0:
            (x, y), radius = cv2.minEnclosingCircle(best_cnt)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(thresh2, center, radius, (0, 0, 255), 2)

            # Note: Using circle here
            # If it's too small don't follow
            # If it's too big don't follow
            # Otherwise map to between 0.2 and 0.4 throttle
            min_radius = Constants.max_radius
            max_radius = Constants.min_radius
            min_throttle = Constants.min_throttle
            max_throttle = Constants.max_throttle
            if radius > min_radius and radius < max_radius:
                throttle = max_throttle - ((max_throttle - min_throttle) * ((radius - min_radius) / (max_radius - min_radius)))

        self.actuator_mixer.update(throttle, angle)
        print('\n CAR: cx: {}, cy: {}, max_area: {}, angle: {:+04.2f}, throttle: {:+04.2f}'.format(
            cx, cy, max_area, angle, throttle), end='')

        cv2.imshow("Frame", blur)
        cv2.imshow('thresh', thresh2)
        # show the frame
        key = cv2.waitKey(1) & 0xFF

        return
