import os
import threading
import tkinter as tk
import types

import ttkbootstrap.constants

import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from threading import Thread
from PIL import ImageTk, Image
from model.main_model import MainModel
from controller.main_controller import MainController
from husky_msgs.msg import HuskyStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tkinter import filedialog as fd
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseGoal,MoveBaseResult, MoveBaseActionResult, MoveBaseAction
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from tkintermapview import TkinterMapView
import time
from enum import Enum
import json


class ActionType(Enum):
    MoveTo = 1
    DeployAidKit = 2
    Explore = 3
    SeekForHuman = 4
    RotateBy = 5
    MoveBy = 6
    MoveThrough = 7


class ActionModel(tk.Button):
    def __init__(self, parent_frame, action_type, row, column, main_model):
        self.data = {
            'MoveTo': {'x': None, 'y': None},
            'DeployAidKit': {},
            'Explore': {},
            'SeekForHuman': {},
            'RotateBy': {'direction': None, 'angle': None},
            'MoveBy': {'x': None, 'y': None},
            'MoveThrough': {'waypoints': []}
        }
        self.main_model = main_model
        self.action_type = action_type
        self.parent_frame = parent_frame
        self.create_popup_window()
        super().__init__(parent_frame, text=self.action_type.name, width=10, height=5)
        self.grid(row=row, column=column)
        self.bind("<Button-1>", self.left_on_click)
        self.bind("<Button-3>", self.right_on_click)

    def left_on_click(self, event):
        self.popup_window.deiconify()

    def right_on_click(self, event):
        self.main_model.remove_action(self)
        self.popup_window.destroy()
        self.destroy()

    def move_to(self):
        """Move to given point"""
        pass

    def deploy_aid_kit(self):
        pass

    def explore(self):
        """Explore and map area"""
        pass

    def seek_for_human(self):
        pass

    def rotate_by_angle(self):
        """Turn right or left by given angle"""
        pass

    def move_by(self):
        """Move from current position by given value of meters in given direction"""
        pass

    def move_through_waypoints(self):
        pass

    def create_popup_window(self):
        self.popup_window = tk.Toplevel(self.parent_frame)
        self.popup_window.title(f"Configure {self.action_type.name} action")
        self.popup_window.protocol("WM_DELETE_WINDOW", self.popup_window.withdraw)
        self.popup_window.withdraw()
        data_label = ttk.LabelFrame(self.popup_window, text="Action data")
        data_label.pack()
        if self.action_type == ActionType.MoveTo:
            self.data['MoveTo']['x'] = ttk.StringVar(value="0.0")
            self.data['MoveTo']['y'] = ttk.StringVar(value="0.0")
            ttk.Label(data_label, text="Coordinates").grid(row=0, columnspan=2, column=0)
            ttk.Label(data_label, text="X").grid(row=1, column=0)
            ttk.Label(data_label, text="Y").grid(row=1, column=1)
            ttk.Entry(data_label, textvariable=self.data['MoveTo']['x']).grid(row=2, column=0)
            ttk.Entry(data_label, textvariable=self.data['MoveTo']['y']).grid(row=2, column=1)

        elif self.action_type == ActionType.DeployAidKit:
            ttk.Button(data_label, text=":)", command=lambda: print("2137")).pack()
        elif self.action_type == ActionType.Explore:
            ttk.Button(data_label, text=":)", command=lambda: print("2137")).pack()
        elif self.action_type == ActionType.SeekForHuman:
            ttk.Button(data_label, text=":)", command=lambda: print("2137")).pack()
        elif self.action_type == ActionType.RotateBy:
            self.data['RotateBy']['direction'] = ttk.StringVar(value="0.0")
            self.data['RotateBy']['angle'] = ttk.StringVar(value="0.0")
            ttk.Label(data_label, text="Rotate data").grid(row=0, columnspan=2, column=0)
            ttk.Label(data_label, text="Direction").grid(row=1, column=0)
            ttk.Label(data_label, text="Angle").grid(row=1, column=1)
            combobox = ttk.Combobox(data_label, state='readonly',values=('Left','Right'), textvariable= self.data['RotateBy']['direction'])
            combobox.set('Left')
            combobox.grid(row=2, column=0)
            ttk.Entry(data_label, textvariable= self.data['RotateBy']['angle']).grid(row=2, column=1)
        elif self.action_type == ActionType.MoveBy:
            self.data['MoveBy']['x'] = ttk.StringVar(value="0.0")
            self.data['MoveBy']['y'] = ttk.StringVar(value="0.0")
            ttk.Label(data_label, text="Move by").grid(row=0, columnspan=2, column=0)
            ttk.Label(data_label, text="X").grid(row=1, column=0)
            ttk.Label(data_label, text="Y").grid(row=1, column=1)
            ttk.Entry(data_label, textvariable=self.data['MoveBy']['x']).grid(row=2, column=0)
            ttk.Entry(data_label, textvariable=self.data['MoveBy']['y']).grid(row=2, column=1)
        elif self.action_type == ActionType.MoveThrough:
            ttk.Button(data_label, text=":)", command=lambda: print("2137")).pack()

    def get_data(self):
        return self.data

    def callback(self, data):
        print(data.status)
        self.result = data.status

    def prepare_move_to_data(self):
        current_time = rospy.rostime.get_rostime()
        action_goal = MoveBaseActionGoal()
        action_goal.header.frame_id = ''
        action_goal.header.stamp = current_time
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = current_time
        print(goal.target_pose.header.stamp)
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.pose.position.x = float(self.data['MoveTo']['x'].get())
        goal.target_pose.pose.position.y = float(self.data['MoveTo']['y'].get())
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = -0.9664807502798402
        goal.target_pose.pose.orientation.w = 0.2567390880612401
        action_goal.goal = goal
        action_goal.goal_id.id = 'JP2'
        action_goal.goal_id.stamp = current_time
        return goal


    def execute(self):
        if self.action_type == ActionType.MoveTo:
            self.result = 0
            self.goal_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            self.goal_client.wait_for_server()
            goal = self.prepare_move_to_data()
            self.goal_client.send_goal(goal)
            self.goal_client.wait_for_result()
            print("Finito")

        elif self.action_type == ActionType.DeployAidKit:
            pass
        elif self.action_type == ActionType.Explore:
            pass
        elif self.action_type == ActionType.SeekForHuman:
            pass
        elif self.action_type == ActionType.RotateBy:
            pass
        elif self.action_type == ActionType.MoveBy:
            pass
        elif self.action_type == ActionType.MoveThrough:
            pass
