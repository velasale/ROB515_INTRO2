#!/usr/bin/python3
# coding=utf8
import sys

sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import numpy as np
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

import rossros as rr


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()


class ArmSensing():
    """ This class and its methods return the image sensed by the camera
    resized and with a crosshair"""

    def __init__(self, task, my_camera):
        self.task = task
        self.my_camera = my_camera

    def function(self, msg):
        self.task = msg

        # print('\nThread: Camera Sensing...')
        image = self.my_camera.frame
        # image = np.ones((640, 480, 3))

        if image is not None:
            self.task.img = image.copy()
            self.task.frame_gb = self.cross_hair()

        return self.task

    def cross_hair(self):
        """Applies CrossHair to image """
        img_h, img_w = self.task.img.shape[:2]
        cv2.line(self.task.img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.task.img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(self.task.img, self.task.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        return frame_gb


class ArmInterpreter():
    """ This class and its methods return the x,y location of the object,
    and also decide"""

    def __init__(self, task):
        self.task = task
        self.area_max = 0
        self.areaMaxContour = 0
        self.rect = None
        self.box = None
        self.count = 0

    def function(self, msg):
        print('Thread: Camera Interpreting...')

        self.task = msg

        frame_lab = self.filter(self.task.frame_gb)

        self.area_max = 0
        self.areaMaxContour = 0
        if not self.task.start_pick_up:

            # Sweep all colors and find the largest contour
            self.findContour(frame_lab)

            if self.area_max > 2500:
                # Place label and rectangle around contour
                self.labelContour()

                self.task.track = True

                # Decide to move if object is steady within a distance and after a period of time
                distance_threshold = 0.3
                time_threshold = 1.5
                self.decideToMove(distance_threshold, time_threshold)

        return self.task

    def sense_load(self, msg):
        """State Machine to perform cargo sensing"""

        print('Thread: Camera Interpreting... ', self.task.act_flag, self.count)
        self.task = msg

        frame_lab = self.filter(self.task.frame_gb)
        self.area_max = 0
        self.areaMaxContour = 0

        # Wait for car to do a loop with cargo
        if self.task.act_flag == 'Waiting to see cargo':

            self.findContour(frame_lab)
            print('Max sensed area: ', self.area_max, )

            if self.area_max > 2500 and self.count == 0:
                self.count += 1
                # Place label and rectangle around contour
                self.labelContour()
            elif self.count == 1 and self.area_max < 2500:
                self.task.sense_flag = 'Blocking Road'
                self.task.act_flag = 'idle'

        # Proceed to Wait to car to stop
        elif self.task.act_flag == 'Waiting for car to stop':

            self.findContour(frame_lab)
            print('Max sensed area: ', self.area_max, )

            if self.area.max > 2500:
                self.labelContour()

                distance_threshold = 0.3
                time_threshold = 1.5
                self.decideToMove(distance_threshold, time_threshold)

                if self.task.start_pick_up:
                    self.task.sense_flag = 'Picking cargo from car'
                    self.task.act_flag = 'idle'

        return self.task

    def filter(self, frame_gb):
        """Applies filter and Draws a rectangle"""

        if self.task.get_roi and self.task.start_pick_up:
            self.task.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.task.roi, self.task.size)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert image to lab space

        return frame_lab

    def getAreaMaxContour(self, contours):
        """ Find the contour with the largest area
        parameter is list of contours to compare"""
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        # iterate over all contours
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate area of contour
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only when the area > 300, the contour of the largest area is valid to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # returns largest contour

    def findContour(self, frame_lab):

        for i in color_range:  # color range comes from LABconfig.py
            if i in self.task.target_color:
                self.task.detect_color = i

                # Perform bitwise operations on original image and mask
                frame_mask = cv2.inRange(frame_lab, color_range[self.task.detect_color][0],
                                         color_range[self.task.detect_color][1])
                # Open Operation
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                # Close Operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                # Find the outline
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                # Find the largest contour
                self.areaMaxContour, self.area_max = self.getAreaMaxContour(contours)

    def decideToMove(self, d_threshold, t_threshold):
        # Compare the last coordinates to determine whether to move
        distance = math.sqrt(
            pow(self.task.world_x - self.task.last_x, 2) + pow(self.task.world_y - self.task.last_y, 2))
        self.task.last_x, self.task.last_y = self.task.world_x, self.task.world_y

        # Cumulative judgement
        if self.task.action_finish:
            if distance < d_threshold:  # originally 0.3
                self.task.center_list.extend((self.task.world_x, self.task.world_y))
                self.task.count += 1
                if self.task.start_count_t1:
                    self.task.start_count_t1 = False
                    self.task.t1 = time.time()
                if time.time() - self.task.t1 > t_threshold:
                    self.task.rotation_angle = self.rect[2]
                    self.task.start_count_t1 = True
                    self.task.world_X, self.task.world_Y = np.mean(
                        np.array(self.task.center_list).reshape(self.task.count, 2), axis=0)
                    self.task.count = 0
                    self.task.center_list = []
                    self.task.start_pick_up = True
            else:
                self.task.t1 = time.time()
                self.task.start_count_t1 = True
                self.task.count = 0
                self.task.center_list = []

    def labelContour(self):

        self.rect = cv2.minAreaRect(self.areaMaxContour)
        self.box = np.int0(cv2.boxPoints(self.rect))

        self.task.roi = getROI(self.box)
        self.task.get_roi = True

        # Get the coordinates of the center of the block
        img_centerx, img_centery = getCenter(self.rect, self.task.roi, self.task.size, square_length)
        # Convert to real world coordinates
        self.task.world_x, self.task.world_y = convertCoordinate(img_centerx, img_centery, self.task.size)

        # Draw Contours
        cv2.drawContours(self.task.img, [self.box], -1, self.task.range_rgb[self.task.detect_color], 2)
        cv2.putText(self.task.img, '(' + str(self.task.world_x) + ',' + str(self.task.world_y) + ')',
                    (min(self.box[0, 0], self.box[2, 0]), self.box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.task.range_rgb[self.task.detect_color], 1)  # draw center point


class ArmController():
    """Given the x,y location of an object, the controller takes the object into
    the respective color bin"""

    def __init__(self, task):
        self.task = task

        # Placement coordinates of wooden blocks of different colors (x,y,z)
        self.coordinate = {
            'red': (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5, 1.5),
            'blue': (-15 + 0.5, 0 - 0.5, 1.5),
        }

    def function(self, msg):
        print("\nThread: Arm Controller:", self.task.first_move)
        self.task = msg

        # if self.task.__isRunning:
        # When an object is first detected
        if self.task.first_move and self.task.start_pick_up:
            # Make initial move to bring arm close to object
            self.initialMove()

        # Object not detected for the first time
        elif not self.task.first_move and not self.task.unreachable:
            self.set_rgb(self.task.detect_color)

            # If tracking phase
            if self.task.track:
                # Stop and exit flag detection
                AK.setPitchRangeMoving((self.task.world_x, self.task.world_y - 2, 5), -90, -90, 0, 20)
                time.sleep(0.02)
                self.task.track = False

            # If object hasnt moved for a while
            if self.task.start_pick_up:

                self.task.action_finish = False

                pick_coords = [self.task.world_X, self.task.world_Y]
                place_coords = [self.coordinate[self.task.detect_color][0],
                                self.coordinate[self.task.detect_color][1],
                                self.coordinate[self.task.detect_color][2]]
                self.pickAndPlace(pick_coords, place_coords)
                self.initialPose()
                time.sleep(1.5)
                self.resetVariables()

            else:
                time.sleep(0.01)

        return self.task

    def load_car(self, msg):
        """State Machine to perform cargo swapping"""

        print("\nThread: Arm Controller:", self.task.sense_flag)
        self.task = msg

        if self.task.sense_flag == 'Waiting to see cargo':
            self.initialPose()
            time.sleep(0.1)
            self.task.sense_flag = 'idle'

        # Place blocking-block on road -> red
        if self.task.sense_flag == 'Blocking Road':

            pick_coords = [self.coordinate['red'][0],
                            self.coordinate['red'][1],
                            self.coordinate['red'][2]]

            place_coords = [0, 20, 2]
            self.pickAndPlace(pick_coords, place_coords)
            time.sleep(0.1)
            self.initialPose()
            time.sleep(0.1)
            self.task.act_flag = 'Waiting for car to stop'
            self.task.sense_flag = 'idle'

        # Pick-up block from car and bring it to its respective bin
        elif self.task.sense_flag == 'Picking cargo from car':

            pick_coords = [4, 20, 2] # --> to replace with sensed coordinates

            place_coords = [self.coordinate['green'][0],
                            self.coordinate['green'][1],
                            self.coordinate['green'][2]]

            self.pickAndPlace(pick_coords, place_coords)
            self.task.act_flag = 'Swapping cargo into car'
            self.task.sense_flag = 'idle'

        # Then pick-up the other block and put it on top of the car
        elif self.task.act_flag == 'Swapping cargo into car':

            pick_coords = [self.coordinate['blue'][0],
                           self.coordinate['blue'][1],
                           self.coordinate['blue'][2]]

            place_coords = [4, 20, 2] # --> to replace with sensed coordinates
            self.pickAndPlace(pick_coords, place_coords)
            self.task.act_flag = 'Removing Block'
            self.task.sense_flag = 'idle'

        # Remove blocking-block on road -> red
        elif self.task.act_flag == 'Removing Block':
            place_coords = [self.coordinate['red'][0],
                           self.coordinate['red'][1],
                           self.coordinate['red'][2]]

            pick_coords = [0, 20, 2]
            self.pickAndPlace(pick_coords, place_coords)
            self.task.act_flag = ' Waiting to see cargo'
            self.task.sense_flag = 'idle'

        return self.task

    def resetVariables(self):
        "Reset perception and actuation variables"
        self.task.detect_color = 'None'
        self.task.first_move = True
        self.task.get_roi = False
        self.task.action_finish = True
        self.task.start_pick_up = False
        self.set_rgb(self.task.detect_color)

    def initialMove(self):
        """Performs a first approach to the localized object"""

        self.task.action_finish = False
        self.set_rgb(self.task.detect_color)

        # Do not fill the running time parameter, adaptive running time
        result = AK.setPitchRangeMoving((self.task.world_X, self.task.world_Y - 2, 5), -90, -90, 0)
        if result == False:
            self.task.unreachable = True
        else:
            self.task.unreachable = False

        # The third time of the return is time
        time.sleep(result[2] / 1000)
        self.task.start_pick_up = False
        self.task.first_move = False
        self.task.action_finish = True

    def set_rgb(self, color):
        """Set the rgb light color of the expansion board to match the color to be tracked"""
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()

    def initialPose(self):
        """ Initial Position"""
        Board.setBusServoPulse(1, self.task.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def pickAndPlace(self, pick_coords, place_coords):

        # Open Paws
        Board.setBusServoPulse(1, self.task.servo1 - 280, 500)

        # Rotate Paws
        servo2_angle = getAngle(pick_coords[0], pick_coords[1], self.task.rotation_angle)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.8)

        # Lower Altitude
        AK.setPitchRangeMoving((pick_coords[0], pick_coords[1], 2), -90, -90, 0, 1000)
        time.sleep(2)

        # Close paws
        Board.setBusServoPulse(1, self.task.servo1, 500)
        time.sleep(1)

        # Raise arm
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((pick_coords[0], pick_coords[1], 12), -90, -90, 0, 1000)
        time.sleep(1)

        # Take to the target position
        result = AK.setPitchRangeMoving((place_coords[0], place_coords[1], 12), -90, -90, 0)
        time.sleep(result[2] / 1000)

        # Rotate to align block with bin
        servo2_angle = getAngle(place_coords[0], place_coords[1], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)

        # Lower block closer
        AK.setPitchRangeMoving((place_coords[0], place_coords[1], place_coords[2] + 3), -90, -90, 0, 500)
        time.sleep(0.5)

        # Final approach
        # Step 9:
        AK.setPitchRangeMoving(place_coords, -90, -90, 0, 1000)
        time.sleep(0.8)

        # Open Paws and drop object
        Board.setBusServoPulse(1, self.task.servo1 - 280, 500)
        time.sleep(0.8)

        # Raise Arm
        AK.setPitchRangeMoving((place_coords[0], place_coords[1], 12), -90, -90, 0, 800)
        time.sleep(0.8)



class ImageVisualizer():

    def __init__(self):
        ...

    def function(self, msg):
        # print("Thread: Displaying image")
        cv2.imshow('Frame', msg.img)
        cv2.waitKey(1)


class ArmTask():

    def __init__(self):
        self.count = 0
        self.track = False
        self.roi = ()
        self.get_roi = False
        self.unreachable = False
        self.first_move = True
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.t1 = 0
        self.center_list = []
        self.img = None
        self.frame_gb = None

        # General Variables
        self.__isRunning = True
        self.target_color = ('green',)
        self._stop = False

        # Perception Parameters
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255)
        }
        self.size = (640, 480)

        # Actuation Variables
        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.last_x, self.last_y = 0, 0
        self.servo1 = 500

        # State flow variables
        self.act_flag = 'Waiting to see cargo'
        self.sense_flag = 'Waiting to see cargo'

    def reset(self):
        ...
        # TODO


def main():
    """ Refactoring of Perception and Actuation Code"""

    task = ArmTask()

    # Initiate Camera Object
    my_camera = Camera.Camera()
    my_camera.camera_open()
    time.sleep(2)

    # --- PART 1 ---
    # Instances of Sensor, interpreter and controller
    sensor = ArmSensing(task, my_camera)
    interpreter = ArmInterpreter(task)
    controller = ArmController(task)
    display = ImageVisualizer()

    # Instances of Buses
    print('Instances of buses')
    bSensor = rr.Bus(sensor.function(task), "Camera Sensor Bus")
    bInterpreter = rr.Bus(interpreter.sense_load(bSensor.message), "Interpreter Sensor Bus")
    bController = rr.Bus(controller.load_car(bInterpreter.message), "Controller Sensor Bus")
    bTerminate = rr.Bus(0, "Termination Bus")

    # --- PART 2 ---
    print('Wrapping functions')
    # Wrap Sensor, Interpreter and Controller function into RossROS objectsz

    wSensor = rr.ConsumerProducer(
        sensor.function,
        bController,
        bSensor,
        0.01,
        bTerminate,
        "Read Camera Sensor")

    wInterpreter = rr.ConsumerProducer(
        interpreter.sense_load,
        bSensor,
        bInterpreter,
        0.01,
        bTerminate,
        "Interpret Camera")

    wController = rr.ConsumerProducer(
        controller.load_car,
        bInterpreter,
        bController,
        1.0,
        bTerminate,
        "Controlling Arm")

    wDisplay = rr.Consumer(
        display.function,
        bInterpreter,
        0.01,
        bTerminate,
        "Display Image")

    # --- PART 3 ---
    # Create RossROS Timer Object
    terminationTimer = rr.Timer(
        bTerminate,
        10,
        0.1,
        bTerminate,
        "Termination Timer")

    # --- PART 4 ---
    # Concurrent Execution
    producer_consumer_list = [wSensor,
                              wInterpreter,
                              wController,
                              wDisplay
                              ]

    # Execute the list of produces-consumers concurrently
    print('running concurrent\n\n')
    rr.runConcurrently(producer_consumer_list)


if __name__ == '__main__':
    main()