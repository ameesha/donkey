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

cfg = dk.config.parse_config('~/mydonkey/vehicle.ini')
constants = cfg['constants']

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (constants.getint('res_length'), constants.getint('res_width'))
camera.framerate = constants.getint('frame_rate')
camera.hflip = True

rawCapture = PiRGBArray(camera, size=(constants.getint('res_length'), constants.getint('res_width')))

# allow the camera to warmup
time.sleep(0.1)


class BaseVehicle:
    def __init__(self, actuator_mixer: PWMSteeringActuator=None):
        # these need to be updated when vehicle is defined
        self.actuator_mixer = actuator_mixer

    def start(self):
        start_time = time.time()
        angle = 0.
        throttle = 0.

        self.capture_frame()

    def capture_frame(self):
        # capture frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            t = Thread(self.calculate_throttle_and_angle(image), args=())
            t.daemon = True
            t.start()

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)

    def calculate_throttle_and_angle(self, image):
        blur = cv2.blur(image, (constants.getint('blur_anchor_x'), constants.getint('blur_anchor_y')))
            
        # pink
        lower = np.array([50, 20, 90], dtype="uint8")
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
        angle = ((constants.getint('angle_blob') - cx) / constants.getint('angle_blob') * 2) - 1

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
            min_radius = constants.getint('max_radius')
            max_radius = constants.getint('min_radius')
            min_throttle = constants.getfloat('min_throttle')
            max_throttle = constants.getfloat('max_throttle')
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
