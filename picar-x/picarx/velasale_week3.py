import picarx_improved
import time
import csv
import time
import statistics as st
import os
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np


class GrayInterpreter():

    def __init__(self, dark_threshold=600, light_threshold=1400, polarity="line_darker"):

        self.dark_threshold = dark_threshold
        self.light_threshold = light_threshold
        self.polarity = polarity

        self.left_signal = []
        self.center_signal = []
        self.right_signal = []
        self.means = []

        # Moving average window
        self.window = 10

        # Normalizing value
        self.normalizer = 0.6

    def sharp_edge(self, data: list):
        """
        Method to smooth the ADC readings with a moving average, and
        Method to identify a change in the sensor values
        @type data: list of three ADC signals
        """

        self.left_signal.append(int(data[0]))
        self.center_signal.append(int(data[1]))
        self.right_signal.append(int(data[2]))

        if len(self.left_signal) > self.window:
            self.left_signal.pop(0)
            self.center_signal.pop(0)
            self.right_signal.pop(0)

        mean1 = int(st.mean(self.left_signal))
        mean2 = int(st.mean(self.center_signal))
        mean3 = int(st.mean(self.right_signal))

        # Mean values of the ADC channels
        means = [int(mean1), int(mean2), int(mean3)]

        # --- Approach 1: Calculates the centroid of the means with respect to the center
        centroid = (mean3 - mean1) / (mean1 + mean2 + mean3)
        centroid = centroid / self.normalizer

        # --- Approach 2: Trying to make it more robust for different light conditions
        # It calculates the centroid w.r.t the min and max readings, and therefore is more robust
        # different lighting conditions
        min_reading = min(means)
        max_reading = max(means)
        print(min_reading)

        n_mean1 = (mean1 - min_reading) / (max_reading - min_reading)
        n_mean2 = (mean2 - min_reading) / (max_reading - min_reading)
        n_mean3 = (mean3 - min_reading) / (max_reading - min_reading)

        n_centroid = (n_mean3 - n_mean1) / (n_mean1 + n_mean2 + n_mean3)
        n_centroid = n_centroid

        return means, centroid, n_centroid

    def position(self):
        """
        This method outputs the location of the robot relative to the line as a value
        on the interval [-1, 1], with positive values being to the left of the robot.
        """


class GrayController():

    def __init__(self, scale_factor=20):

        # K for a proportional controller, it maps [-1,1] into [25,-25]deg
        self.scale_factor = scale_factor

    def steer_towards_line(self, error):
        return self.scale_factor * error


class PicarCamera():
    
    def __init__(self):
        self.color_dict = {'red': [0, 4], 'orange': [5, 18], 'yellow': [22, 37], 'green': [42, 85], 'blue': [92, 110],
                      'purple': [115, 165],
                      'red_2': [165, 180]}  # Here is the range of H in the HSV color space represented by the color
        self.kernel_5 = np.ones((5, 5), np.uint8)  # Define a 5×5 convolution kernel with element values of all 1.

    def color_detect(self, img, color_name):

        # The blue range will be different under different lighting conditions and can be adjusted flexibly.  H: chroma, S: saturation v: lightness
        resize_img = cv2.resize(img, (160, 120),
                                interpolation=cv2.INTER_LINEAR)  # In order to reduce the amount of calculation, the size of the picture is reduced to (160,120)
        hsv = cv2.cvtColor(resize_img, cv2.COLOR_BGR2HSV)  # Convert from BGR to HSV
        color_type = color_name

        mask = cv2.inRange(hsv, np.array([min(self.color_dict[color_type]), 60, 60]), np.array(
            [max(self.color_dict[color_type]), 255,
             255]))  # inRange()：Make the ones between lower/upper white, and the rest black
        if color_type == 'red':
            mask_2 = cv2.inRange(hsv, (self.color_dict['red_2'][0], 0, 0), (self.color_dict['red_2'][1], 255, 255))
            mask = cv2.bitwise_or(mask, mask_2)

        morphologyEx_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_5,
                                            iterations=1)  # Perform an open operation on the image

        # Find the contour in morphologyEx_img, and the contours are arranged according to the area from small to large.
        _tuple = cv2.findContours(morphologyEx_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # compatible with opencv3.x and openc4.x
        if len(_tuple) == 3:
            _, contours, hierarchy = _tuple
        else:
            contours, hierarchy = _tuple

        color_area_num = len(contours)  # Count the number of contours

        if color_area_num > 0:
            for i in contours:  # Traverse all contours
                x, y, w, h = cv2.boundingRect(
                    i)  # Decompose the contour into the coordinates of the upper left corner and the width and height of the recognition object

                print("X location is:  ", (x + w / 2))

                # Draw a rectangle on the image (picture, upper left corner coordinate, lower right corner coordinate, color, line width)
                if w >= 8 and h >= 8:  # Because the picture is reduced to a quarter of the original size, if you want to draw a rectangle on the original picture to circle the target, you have to multiply x, y, w, h by 4.
                    x = x * 4
                    y = y * 4
                    w = w * 4
                    h = h * 4
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw a rectangular frame
                    cv2.putText(img, color_type, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255),
                                2)  # Add character description

        return img, mask, morphologyEx_img
    
    
def sample_code(px):
    """ Sample code from https://docs.sunfounder.com/projects/picar-x/en/latest/python/python_move.html
    """
    try:

        px.forward(30)
        time.sleep(0.5)
        for angle in range(0,35):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(-35,0):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        px.forward(0)
        time.sleep(1)

        for angle in range(0,35):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(-35,0):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(0,35):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)
        for angle in range(-35,0):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)

    finally:
        px.forward(0)


def steering(px):
    for angle in range(0, 35):
        px.set_dir_servo_angle(angle)
        time.sleep(0.01)
    time.sleep(2)
    for angle in range(35, -35, -1):
        px.set_dir_servo_angle(angle)
        time.sleep(0.01)
    time.sleep(2)
    for angle in range(-35, 0):
        px.set_dir_servo_angle(angle)
        time.sleep(0.01)


def parallel_parking(px, side):
    # Move Forward a bit
    px.set_dir_servo_angle(0)
    px.forward(5)
    time.sleep(1)

    px.stop()
    time.sleep(1)

    if side == "L":
        factor = 1
    elif side == "R":
        factor = -1

    # Move Backward
    # Steer wheels in one direction
    px.set_dir_servo_angle(factor * -1 * px.CAR_MAX_STEERING_ANGLE)
    px.forward(-5)
    time.sleep(0.8)
    # Steer wheels in the opposite direction
    px.set_dir_servo_angle(factor * px.CAR_MAX_STEERING_ANGLE)
    px.forward(-5)
    time.sleep(0.8)

    px.set_dir_servo_angle(0)
    px.stop()


def k_turning(px, side):
    """
    Three point turn (sometimes called Y-turn, K-turn, or broken U-turn)
    """
    factor = 1
    if side == "L":
        factor = 1
    elif side == "R":
        factor = -1

    # 1st Point: Steer left and move forward
    px.set_dir_servo_angle(-1 * px.CAR_MAX_STEERING_ANGLE * factor)
    px.forward(5)
    time.sleep(1)
    px.stop()
    time.sleep(1)

    # 2nd Point: Steer Right and move backward
    px.set_dir_servo_angle(px.CAR_MAX_STEERING_ANGLE * factor)
    px.forward(-5)
    time.sleep(1)
    px.stop()
    time.sleep(1)

    # 3rd Point: Steer Left and move forward
    px.set_dir_servo_angle(-1 * px.CAR_MAX_STEERING_ANGLE * factor)
    # Gradually adjust the steering angle to zero
    if side == "R":
        while px.dir_current_angle > 0:
            px.forward(5)
            time.sleep(0.25)
            angle = px.dir_current_angle - 5
            px.set_dir_servo_angle(angle)
    elif side == "L":
        while px.dir_current_angle < 0:
            px.forward(5)
            time.sleep(0.25)
            angle = px.dir_current_angle + 5
            px.set_dir_servo_angle(angle)


    time.sleep(2)
    px.stop()


def fw_bw(px, speed, angle):

    px.set_dir_servo_angle(angle)
    # First Move Forward
    goal_speed = speed
    speed = 0
    dt = 0.005

    print("------------- Phase 1: Accelerating -------------")
    while speed < goal_speed:
        print("Current Speed: ", str(speed))
        speed += px.CAR_ACCEL * dt
        px.forward(speed)
        time.sleep(dt)

    print("-------- Phase 2: Coasting at goal speed --------")
    time.sleep(3)

    print("------------- Phase 3: Decelerating -------------")
    while speed > 0:
        print("Current Speed: ", str(speed))
        speed -= px.CAR_ACCEL * dt
        px.forward(speed)
        time.sleep(dt)

    px.stop()
    time.sleep(4)

    # Then Move Backward
    goal_speed = -40
    speed = 0

    print("------------- Phase 1: Accelerating -------------")
    while speed > goal_speed:
        print("Current Speed: ", str(speed))
        speed -= px.CAR_ACCEL * dt
        px.forward(speed)
        time.sleep(dt)

    print("-------- Phase 2: Coasting at goal speed --------")
    time.sleep(3)

    while speed < 0:
        print("------------- Phase 3: Decelerating -------------")
        speed += px.CAR_ACCEL * dt
        px.forward(speed)
        time.sleep(dt)

    px.stop()


def week_2(px):
    while True:
        # Ask user
        side = input("In what direction do you want to maneuver (L: Left, R: Right)\n")
        maneuver = input("Enter desired maneuver (1: k-turning, 2:parallel, 3:fw_bw)\n")

        if side == "L":
            if maneuver == "1":
                k_turning(px, side)
            elif maneuver == "2":
                parallel_parking(px, side)
            elif maneuver == "3":
                fw_bw(px, 40, -5)
            else:
                pass
        else:
            if maneuver == "1":
                k_turning(px, side)
            elif maneuver == "2":
                parallel_parking(px, side)
            elif maneuver == "3":
                fw_bw(px, 40, 5)
            else:
                pass


def week_3(px):
    """ Sensor-control integration"""

    # Instance of an Interpreter
    photosensors = GrayInterpreter()
    # Instance of a Controller
    control = GrayController()

    e_time = 0
    start = time.time()

    while e_time < 4:
        os.system('clear')
        data = px.get_grayscale_data()

        # Step 1: Read and Interpret
        adc_means, adc_centroid, adc_ncentroid = photosensors.sharp_edge(data)
        steer_angle = control.steer_towards_line(adc_ncentroid)

        print("\nThe signals are: ", adc_means)
        # print("The center line is located at: %.2f" % adc_centroid)
        print("The n_center line is located at: %.2f" % adc_ncentroid)

        # Step 2: Control
        px.set_dir_servo_angle(steer_angle)
        print("The commanded steer angle is: %.2f" % steer_angle)
        px.forward(1)

        e_time = time.time() - start
        

if __name__ == "__main__":
    px = picarx_improved.Picarx()

    cam_class = PicarCamera()

    # week_2(px)
    # week_3(px)

    with PiCamera() as camera:
        print("start color detect")
        camera.resolution = (640, 480)
        camera.framerate = 24
        rawCapture = PiRGBArray(camera, size=camera.resolution)
        time.sleep(2)

        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):  # use_video_port=True
            img = frame.array
            img, img_2, img_3 = cam_class.color_detect(img, 'purple')  # Color detection function
            cv2.imshow("video", img)  # OpenCV image show
            cv2.imshow("mask", img_2)  # OpenCV image show
            cv2.imshow("morphologyEx_img", img_3)  # OpenCV image show
            rawCapture.truncate(0)  # Release cache

            k = cv2.waitKey(1) & 0xFF
            # 27 is the ESC key, which means that if you press the ESC key to exit
            if k == 27:
                break

        print('quit ...')
        cv2.destroyAllWindows()
        camera.close()

