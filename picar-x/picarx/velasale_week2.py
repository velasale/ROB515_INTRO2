import picarx_improved

import logging
import atexit
# from logdecorator import log_on_start , log_on_end , log_on_error

import time
try:
    from robot_hat import *
    from robot_hat import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar-X system (robot is not present). Shadowing hardware calls with substitute functions")
    from sim_robot_hat import *

# For Debugging
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

# from robot_hat import Pin, PWM, Servo, fileDB
# from robot_hat import Grayscale_Module, Ultrasonic
# from robot_hat.utils import reset_mcu
# import time

import os

# reset_mcu()
# time.sleep(0.2)

# user and User home directory
User = os.popen('echo ${SUDO_USER:-$LOGNAME}').readline().strip()
UserHome = os.popen('getent passwd %s | cut -d: -f 6'%User).readline().strip()
# print(User)  # pi
# print(UserHome) # /home/pi
config_file = '%s/.config/picar-x/picar-x.conf'%UserHome


px = picarx_improved.Picarx()
px.forward(10)
time.sleep(2)
px.stop()
