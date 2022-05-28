import os
import threading
import tkinter as tk
import ttkbootstrap as ttk
from math import pi
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from threading import Thread
from PIL import ImageTk, Image
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseResult, MoveBaseActionResult, MoveBaseAction
from enum import Enum


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
            'MoveTo': {'x': ttk.StringVar(value="0.0"), 'y': ttk.StringVar(value="0.0"), 'zo': ttk.IntVar(value=0)},
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

    def validatorxd(self):
        print("dupa")
        self.data['MoveTo']['zo'].set(self.angle_meter.amountusedvar.get())
        self.rotated = self.robot_image.rotate(-1.0*float(self.angle_meter.amountusedvar.get()), center=(247, 297.5))
        self.robot_img = ImageTk.PhotoImage(self.rotated)
        self.canvas.delete(self.image)

        self.image = self.canvas.create_image(250, 250, image=self.robot_img)

        return True

    def create_popup_window(self):
        self.popup_window = tk.Toplevel(self.parent_frame)
        self.popup_window.title(f"Configure {self.action_type.name} action")
        self.popup_window.protocol("WM_DELETE_WINDOW", self.popup_window.withdraw)
        self.popup_window.withdraw()
        data_label = ttk.LabelFrame(self.popup_window, text="Action data")
        data_label.pack()
        if self.action_type == ActionType.MoveTo:
            self.angle_meter = ttk.Meter(data_label, subtextfont='-size 20 -weight bold', textright=u'\N{DEGREE SIGN}',
                                         arcoffset=-90, amounttotal=360, interactive=True,
                                         stripethickness=10, wedgesize=5, padding=50, textleft='-')
            self.angle_meter.grid(row=3, columnspan=3, column=0)
            ttk.Label(data_label, text="Coordinates").grid(row=0, columnspan=3, column=0)
            ttk.Label(data_label, text="X").grid(row=1, column=0)
            ttk.Label(data_label, text="Y").grid(row=1, column=1)
            ttk.Label(data_label, text="Rotation").grid(row=1, column=2)
            ttk.Entry(data_label, textvariable=self.data['MoveTo']['x']).grid(row=2, column=0)
            ttk.Entry(data_label, textvariable=self.data['MoveTo']['y']).grid(row=2, column=1)
            ttk.Entry(data_label, textvariable=self.angle_meter.amountusedvar, validatecommand=self.validatorxd,
                      validate='all').grid(row=2, column=2, padx=75)

            self.robot_image = Image.open('utilities/robot_img.png')
            self.robot_image.resize((100, 100), Image.ANTIALIAS)
            self.robot_img = ImageTk.PhotoImage(self.robot_image)
            self.canvas = tk.Canvas(data_label, bg="blue", height=500, width=500)
            self.canvas.grid(row=4, column=0)
            self.image = self.canvas.create_image(250, 250, image=self.robot_img)



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
            combobox = ttk.Combobox(data_label, state='readonly', values=('Left', 'Right'),
                                    textvariable=self.data['RotateBy']['direction'])
            combobox.set('Left')
            combobox.grid(row=2, column=0)
            ttk.Entry(data_label, textvariable=self.data['RotateBy']['angle']).grid(row=2, column=1)
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
        temp_angle = float(self.data['MoveTo']['zo'].get()) * 2 * pi / 360.0
        print(temp_angle)
        temp_angle = quaternion_from_euler(0, 0, temp_angle)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = temp_angle[2]
        goal.target_pose.pose.orientation.w = temp_angle[3]
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
