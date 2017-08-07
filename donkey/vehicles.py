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
        self.frame = np.zeros(shape=(Constants.res_length, Constants.res_width, 3))

        self.rawCapture = PiRGBArray(self.camera, size=(Constants.res_length, Constants.res_width))

        # allow the camera to warmup
        time.sleep(0.1)
    
    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # capture frames from the camera
        start_time = time.time()
        count = 0
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            count += 1
            now = time.time()
            elapsed_ms = int((now - start_time) * 1000)
            self.frame = frame.array
            self.rawCapture.truncate(0)
            # print('\n Frame, count={}, elapsed_ms={}, fps={}', count, elapsed_ms, 1000 * count/elapsed_ms)
    
    def read(self):
        return self.frame

class ImageProcessingThread:
    def __init__(self, actuator_mixer):
        self.actuator_mixer = actuator_mixer

    def start(self, image):
        # t = Thread(target=self.calculate_throttle_and_angle(image), args=())
        # t.start()
        # return
        self.calculate_throttle_and_angle(image)


    def calculate_throttle_and_angle(self, image):
        blur = cv2.blur(image, (Constants.blur_anchor_x, Constants.blur_anchor_y))
            
        # pink
        lower_light = np.array([40, 5, 85], dtype="uint8")
        upper_light = np.array([70, 35, 115], dtype="uint8")
        thresh_light = cv2.inRange(blur, lower_light, upper_light)

        lower_dark = np.array([0, 0, 35], dtype="uint8")
        upper_dark = np.array([10, 10, 70], dtype="uint8")
        thresh_dark = cv2.inRange(blur, lower_dark, upper_dark)

        lower_mid = np.array([10, 0, 60], dtype="uint8")
        upper_mid = np.array([60, 30, 90], dtype="uint8")
        thresh_mid = cv2.inRange(blur, lower_mid, upper_mid)
        
        lower_pink = np.array([75, 25, 145], dtype="uint8")
        upper_pink = np.array([135, 55, 185], dtype="uint8")
        thresh_pink = cv2.inRange(blur, lower_pink, upper_pink)

        thresh = thresh_light + thresh_dark + thresh_mid + thresh_pink
        thresh2 = thresh.copy()
        
        y = range(0, thresh.shape[0])
        x = range(0, thresh.shape[1])

        (X,Y) = np.meshgrid(x,y)

        cx = int((X*thresh).sum() / thresh.sum())
        cy = int((Y*thresh).sum() / thresh.sum())
        max_area = thresh.sum()
                        
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
        print('\r DSP: cx: {}, cy: {}, max_area: {}, angle: {:+04.2f}, throttle: {:+04.2f}'.format(
            cx, cy, max_area, angle, throttle), end='')

        cv2.imshow("Frame", blur)
        cv2.imshow('thresh', thresh2)
        # show the frame
        key = cv2.waitKey(1) & 0xFF

        return


class BaseVehicle:
    def __init__(self, actuator_mixer: PWMSteeringActuator=None):
        # these need to be updated when vehicle is defined
        self.actuator_mixer = actuator_mixer
        self.camera = CameraStream()
        self.image_processing = ImageProcessingThread(self.actuator_mixer)

    def start(self):
        self.camera.start()

        while(True):
            # print('\n Main process loop')
            self.image_processing.start(self.camera.read())
