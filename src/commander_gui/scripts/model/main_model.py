from model.robot_model import RobotModel
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



class MainModel:
    """
    Module responsible for detecting aruco tags and storing their position
    """

    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.tag_id = None
        self.tag_params = None
        self.model_data = {'GPS': NavSatFix, 'Odom': Odometry}
        self.odom_subscriber = rospy.Subscriber("odometry/filtered", Odometry, self.odom_callback)
        self.gps_subscriber = rospy.Subscriber("gps_point_lat_lng", NavSatFix, self.gps_callback)

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

    def odom_callback(self, data):
        self.model_data['Odom'] = data
