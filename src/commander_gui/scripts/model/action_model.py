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
        self.main_model = main_model
        self.action_type = action_type
        self.parent_frame = parent_frame
        self.create_popup_window()
        super().__init__(parent_frame, text=action_type.name, width=10, height=5)
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

    def execute(self):
        pass
