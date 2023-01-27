import picarx_improved
import time
import csv
import time
import statistics as st


class Gray_interpreter():

    def __init__(self, dark_threshold=1000, light_threshold=1500, polarity=100):

        self.dark_threshold = dark_threshold
        self.light_threshold = light_threshold
        self.polarity = polarity

    def sharp_edge(self, gray_list):
        """
        Method to identify a change in the sensor values
        """

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


class Gray_controller():

    def __init__(self, scale_factor=-25):

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
    trial = Gray_controller()
    while True:

        # -------------- Sense ----------------
        sensors = px.get_grayscale_data()
        time.sleep(0.1)
        print(sensors)
        trend = px.get_line_status(sensors)

        # -------------- Interpreter ----------
        error = 0
        if trend == "left":
            error = +1.0
        elif trend == "right":
            error = -1.0
        elif trend == "forward":
            error = 0
        else:
            print("Stop")

        angle = trial.steer_towards_line(error)

        # -------------- Control --------------
        px.set_dir_servo_angle(angle)
        px.forward(20)


if __name__ == "__main__":
    px = picarx_improved.Picarx()

    # week_2(px)
    # week_3(px)

    signal_1 = []
    signal_2 = []
    signal_3 = []
    signals = []

    while True:

        signal_1 = []
        signal_2 = []
        signal_3 = []
        for n in range(100):
            data = px.get_grayscale_data()
            signal_1.append(data[0])
            signal_2.append(data[1])
            signal_3.append(data[2])

    mean1 = st.mean(signal_1)
    mean2 = st.mean(signal_2)
    mean3 = st.mean(signal_3)

    print(mean1, "  ", mean2, "  ", mean3)

    with open('signals.csv', 'w') as f:
        write = csv.writer(f)
        write.writerow(signals)

    print(signals)
    print("\n\n")
    print(executionTime)

