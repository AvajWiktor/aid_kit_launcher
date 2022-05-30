import rospy
import json

from os import system
import random
import time
import threading
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point as Point_msg
from math import pi, cos, sin, radians, atan
from std_msgs.msg import String
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import *
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from husky_msgs.msg import HuskyStatus
from diagnostic_msgs.msg import DiagnosticArray


class MainModel:
    """
    Module responsible for detecting aruco tags and storing their position
    """

    def __init__(self):
        self.tag_id = None
        self.tag_params = None
        self.model_data = {'GPS': NavSatFix, 'Odom': Odometry, 'Diagnostic': HuskyStatus}
        self.odom_subscriber = rospy.Subscriber("odometry/filtered", Odometry, self.odom_callback)
        self.gps_subscriber = rospy.Subscriber("gps_point_lat_lng", NavSatFix, self.gps_callback)
        self.diagnostic_subscriber = rospy.Subscriber("diagnostics", DiagnosticArray, self.diagnostic_callback)

    def get_data(self):
        return self.model_data

    def read_tag_info(self, tag_id):
        f = open(f"utilities/tags_params.json")
        data = json.load(f)
        self.tag_params = data[tag_id]

    def connect_to_robot(self):
        self.robot_model.connect_to_robot()

    def gps_callback(self, data):
        self.model_data['GPS'] = data
        #rospy.sleep(0.1)

    def odom_callback(self, data):
        self.model_data['Odom'] = data
        #rospy.sleep(0.1)

    def diagnostic_callback(self, data):
        """Jedno wielkie xD
        data.status[0].values[7] - > Left Motor Driver Temp
        data.status[0].values[8] - > Right Motor Driver Temp
        data.status[0].values[9] - > Left Motor Temp
        data.status[0].values[10] - > Right Motor Temp

        data.status[1].values[0] - > Battery Charge [%]
        data.status[1].values[1] - > Battery Capacity [Wh]

        data.status[2].values[0 ... 5] - > Timeout, Lockout, Estop, RosPause, NoBattery, CurrentLimit
        """
        if data.status[0].name == 'husky_node: system_status':
            self.model_data['Diagnostic'] = data
            #print(data.status[0].values)
            rospy.sleep(0.1)
