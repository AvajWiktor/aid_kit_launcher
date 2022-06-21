import rospy
import json
import numpy as np
import cv2
from cv_bridge import CvBridge
from PIL import ImageTk, Image
import sensor_msgs.msg as sensors
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


def detect_red(iemgie):
    print("detektuje czerwony xD\n")
    result = False
    img = iemgie.copy()
    image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # lower boundary RED color range values; Hue (0 - 10)
    # lower boundary RED color range values; Hue (0 - 10)
    # lower boundary RED color range values; Hue (0 - 10)
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])

    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160, 100, 20])
    upper2 = np.array([179, 255, 255])
    lower_mask = cv2.inRange(image, lower1, upper1)
    upper_mask = cv2.inRange(image, lower2, upper2)
    red_mask = lower_mask + upper_mask
    img_res = cv2.bitwise_and(img, img, mask=red_mask)

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cont_list = list()
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        a = w * h
        aspectRatio = float(w) / h

        if aspectRatio >= 0.5 and a > 100:
            approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4:
                cont_list.append(approx)
                # x, y, w, h = cv2.boundingRect(cnt)
                # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                print("wykryto czerwony")
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0
                # cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.line(img, (cX, cY - 10), (cX, cY + 10), (0, 255, 0), 5)
                cv2.line(img, (cX + 10, cY), (cX - 10, cY), (0, 255, 0), 5)
                # cv2.putText(img, 'DAMAGE', [img.shape[0] - 50, img.shape[1] - 150],
                #             cv2.FONT_HERSHEY_SIMPLEX,
                #             0.5, (0, 255, 0), 2, cv2.LINE_AA)
                # [img.shape[0] - 50, img.shape[1] - 150] dostosowac jesli wyjezdza
                # cv2.line(img, (cX-5, cY-5), (cX+5, cY+5), (0, 0, 255), 5)
                result = True
    return img, result


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

    def gps_callback(self, data):
        self.model_data['GPS'] = data
        #rospy.sleep(0.1)

    def odom_callback(self, data):
        self.model_data['Odom'] = data
        #rospy.sleep(0.1)
    
    def get_image(self, camera_id):
        bridge = CvBridge()
        image_message = rospy.wait_for_message(f"/usb_cam_{camera_id}/image_raw", sensors.Image)
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        changed_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        return changed_img
    
    def capture_image(self, camera_id, image_name, type):
        bridge = CvBridge()
        image_message = rospy.wait_for_message(f"/usb_cam_{camera_id}/image_raw", sensors.Image)
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        img = None
        result = False
        if type == "damage":
            img, result = detect_red(cv_image)
        if type == "entrance":
            #img, result = detect_green(cv_image)
            pass
        pixels = np.array(img)
        image = Image.fromarray(pixels.astype('uint8'), 'RGB')
        image.save(f'captured_images/{image_name}.jpg')
        return result
        
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



