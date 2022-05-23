import serial
import subprocess
from math import atan2
import json


class RobotModel:
    def __init__(self, payload=0, tcp_mass=0, robot_speed=0, robot_ip='192.168.0.178'):
        self.move_group = None#
        self.detected_tags = {}  # dict of detected tags
        self.serial_port = None #serial.Serial('/dev/ttyUSB0', 9600, timeout=0.05)
        self.robot_ip = robot_ip

    def connect_to_robot(self):
        rc = subprocess.call("/home/wiktor/Desktop/RobotLaunch.sh")


