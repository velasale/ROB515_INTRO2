import picarx_improved
import time

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


def parallel_parking_left(px):
    # Move Forward a bit
    px.set_dir_servo_angle(0)
    px.forward(5)
    time.sleep(4)

    # Move Backward
    # Steer wheels one direction
    px.set_dir_servo_angle(25)
    px.backward(5)
    time.sleep(2)
    # Steer wheels opposite direction
    px.set_dir_servo_angle(-25)
    px.backward(5)
    time.sleep(2)
    px.stop()


def parallel_parking_right(px):
    pass


def k_turning_left(px):
    """
    Three point turn (sometimes called Y-turn, K-turn, or broken U-turn)
    """
    # 1st Point: Steer left and move forward
    px.set_dir_servo_angle(35)
    px.forward(5)
    time.sleep(4)
    px.stop()

    # 2nd Point: Steer Right and move backward
    px.set_dir_servo_angle(-35)
    px.backward(5)
    time.sleep(4)
    px.stop()

    # 3rd Point: Steer Left and move forward
    px.set_dir_servo_angle(35)
    while px.dir_current_angle > 0:
        px.forward(5)
        time.sleep(0.1)
        angle = px.dir_current_angle - 5
        px.set_dir_servo_angle(angle)

    time.sleep(2)


def k_turning_right(px):
    pass


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


def circle(px):
    pass




if __name__ == "__main__":
    px = picarx_improved.Picarx()
    while True:
        # Ask user
        side = input("In what direction do you want to maneuver (L: Left, R: Right)\n")
        maneuver = input("Enter desired maneuver (1: k-turning, 2:parallel, 3:fw_bw)\n")

        if side == "L":
            if maneuver == "1":
                k_turning_left(px)
            elif maneuver == "2":
                parallel_parking_left(px)
            elif maneuver == "3":
                fw_bw(px, 40, -5)
            else:
                pass
        else:
            if maneuver == "1":
                k_turning_right(px)
            elif maneuver == "2":
                parallel_parking_right(px)
            elif maneuver == "3":
                fw_bw(px, 40, 5)
            else:
                pass

