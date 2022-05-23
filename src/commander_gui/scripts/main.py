#!/usr/bin/env python3
"""
Initial file for entire app
"""
from view import main_view  # , modules_view, settings_bar_view
from model import robot_model
import rospy
if __name__ == '__main__':

    rospy.init_node('Lolek')
    Manipulator = robot_model.RobotModel()
    App = main_view.MainWindowView(Manipulator)
    print("Closed")