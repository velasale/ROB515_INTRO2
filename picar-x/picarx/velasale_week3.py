import picarx_improved
import time
import csv
import time
import statistics as st
import os


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

        # Trying to make it more robust for different light conditions
        min_reading = min(means)
        max_reading = max(means)
        print(min_reading)

        n_mean1 = (mean1 - min_reading) / (max_reading - min_reading)
        n_mean2 = (mean2 - min_reading) / (max_reading - min_reading)
        n_mean3 = (mean3 - min_reading) / (max_reading - min_reading)

        # Calculates the centroid of the means with respect to the center
        centroid = (mean3 - mean1) / (mean1 + mean2 + mean3)
        centroid = centroid / self.normalizer

        n_centroid = (n_mean3 - n_mean1) / (n_mean1 + n_mean2 + n_mean3)
        n_centroid = n_centroid

        return means, centroid, n_centroid


        ## Identify if there is a sharp change in the sensor values
        ## (indicative of an edge)


        ## Using the edge location and sign, to determine both:
        ## (a) whether the system is to the left or right
        ## (b) and whether it is very off-center or only slightly off-center


        ## It should be robust to different lighting conditions, with an option
        ## to have the "target" darker or lighter than the surrounding floor

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
    px.set_dir_servo_angle(factor * -1 * px.car_max_dir_angle)
    px.forward(-5)
    time.sleep(0.8)
    # Steer wheels in the opposite direction
    px.set_dir_servo_angle(factor * px.car_max_dir_angle)
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
    px.set_dir_servo_angle(-1 * px.car_max_dir_angle * factor)
    px.forward(5)
    time.sleep(1)
    px.stop()
    time.sleep(1)

    # 2nd Point: Steer Right and move backward
    px.set_dir_servo_angle(px.car_max_dir_angle * factor)
    px.forward(-5)
    time.sleep(1)
    px.stop()
    time.sleep(1)

    # 3rd Point: Steer Left and move forward
    px.set_dir_servo_angle(-1 * px.car_max_dir_angle * factor)
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
        speed += px.car_accel * dt
        px.forward(speed)
        time.sleep(dt)

    print("-------- Phase 2: Coasting at goal speed --------")
    time.sleep(3)

    print("------------- Phase 3: Decelerating -------------")
    while speed > 0:
        print("Current Speed: ", str(speed))
        speed -= px.car_accel * dt
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
        speed -= px.car_accel * dt
        px.forward(speed)
        time.sleep(dt)

    print("-------- Phase 2: Coasting at goal speed --------")
    time.sleep(3)

    while speed < 0:
        print("------------- Phase 3: Decelerating -------------")
        speed += px.car_accel * dt
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

    # Interpreter
    photosensors = GrayInterpreter()
    # Controller
    control = GrayController()

    while True:
        data = px.get_grayscale_data()

        # Step 1: Read and Interpret
        adc_means, adc_centroid, adc_ncentroid = photosensors.sharp_edge(data)
        steer_angle = control.steer_towards_line(adc_ncentroid)

        os.system('clear')
        print("\nThe signals are: ", adc_means)
        print("The center line is located at: %.2f" % adc_centroid)
        print("The n_center line is located at: %.2f" % adc_ncentroid)

        # Step 2: Control
        px.set_dir_servo_angle(steer_angle)
        print("The commanded steer angle is: %.2f" % steer_angle)
        px.forward(5)


if __name__ == "__main__":
    px = picarx_improved.Picarx()

    # week_2(px)
    week_3(px)

