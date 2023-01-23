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
    px.set_dir_servo_angle(35)
    px.backward(5)
    time.sleep(2)
    # Steer wheels opposite direction
    px.set_dir_servo_angle(-35)
    px.backward(5)
    time.sleep(2)


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


if __name__ == "__main__":
    px = picarx_improved.Picarx()
    while True:
        # Ask user
        maneuver = input("Enter desired maneuver (1: k-turning, 2:parallel)")

        # Execute
        if maneuver == "1":
            k_turning_left(px)
        elif maneuver == "2":
            parallel_parking_left(px)
        else:
            pass
