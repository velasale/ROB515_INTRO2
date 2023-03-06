#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


class ArmPerception():

    def __init__(self, frame, target_color='red'):

        self.target_color = target_color
        self.frame = frame

        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255)
        }

    def getAreaMaxContour(self, contours):
        """ Find the contour with largest area """

        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        # Iterate over all contours
        for contour in contours:
            contour_area_temp = math.fabs(cv2.contourArea(contour))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = contour

        return area_max_contour, contour_area_max






def main():
    """Perception Assignment 1: Set up a simple program that uses this class to identify the location of a block in the pickup
    area and labels it on the video display from the camera."""

    # Start Camera
    camera = Camera.Camera()
    camera.camera.open()

    # Loop
    while True:
        # Picture of what the camera is currently looking
        img = camera.frame
        if img is not None:
            frame = img.copy()
            perception = ArmPerception(frame, tar)






if __name__ == '__main__':
    main()