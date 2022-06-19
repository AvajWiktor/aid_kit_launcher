import datetime
import os
import threading
import tkinter as tk
import types
import serial
import pytz
from datetime import date
import time
import string
import rospy

from cv_bridge import CvBridge
import ttkbootstrap.constants
import numpy as np
import ttkbootstrap as ttk
from model.kml_model import KMLTourGenerator
from ttkbootstrap.constants import *
from threading import Thread
from PIL import ImageTk, Image
from model.main_model import MainModel
from model.action_model import ActionModel, ActionType
from controller.main_controller import MainController
from husky_msgs.msg import HuskyStatus
from aid_kit_launcher.srv import Launcher
from geometry_msgs.msg import Pose, Point, Quaternion
from tkinter import filedialog as fd
from tkintermapview import TkinterMapView
import sensor_msgs.msg as sensors
import json


class MainWindowView:
    def __init__(self):
        self.root = ttk.Window(themename='cyborg', title='Commander GUI')
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.kml_detection_logger = KMLTourGenerator("detection_log")
        self.kml_mission_logger = KMLTourGenerator("mission_log")
        self.kml_waypoint_logger = KMLTourGenerator("waypoint_log")
        self.screen_width = self.root.winfo_screenwidth()
        self.screen_height = self.root.winfo_screenheight()
        self.root.minsize(width=1920, height=1080)
        self.model = MainModel()
        self.event_list = ["Start Human Detection",
                           "Finish Human Detection",
                           "Start going to waypoint",
                           "Finish going to waypoint",
                           "Start deploying aidkit",
                           "Finis deploying aidkit"
                           ]
        self.mission_log_data = ""
        self.camera_list = ["1","2"]
        self.controller = MainController(self.model)
        self.updater = Thread(name='refresher', target=self.update_data)
        self.test_var = tk.IntVar(value=10)
        self.waypoint_name_var = tk.StringVar()

        self.action_list = []
        self.marker_list = {}
        """
        *Code here*
        """
        self.create_data_variables()
        self.create_layout()
        self.create_menu_components()
        self.create_status_components()
        # self.create_map_components()
        self.create_mission_components()
        self.create_serial()
        """
        *Code End*
        """
        self.updater.start()
        self.root.mainloop()

    def create_serial(self):
        self.ser = serial.Serial(
            port='/tmp/printer',
            baudrate=256000)

    def save_target(self):
        self.kml_detection_logger.add_waypoint_wth_image(self.gps_data['Lat'].get(), self.gps_data['Long'].get(), self.target_id.get(), f"captured_images/{self.image_name.get()}.jpg")

    def save_mission_log(self):
        ct = datetime.datetime.now()
        ct = ct.strftime("%d_%m_%Y_%H:%M:%S")
        text_file = open(f"mission_log_{ct}.txt", "w")
        n = text_file.write(self.mission_log_data)
        text_file.close()

    def on_closing(self):
        self.save_mission_log()
        self.kml_mission_logger.finish("mission_log")
        self.kml_detection_logger.finish("detection_log")
        self.kml_waypoint_logger.finish("waypoint_log")
        self.root.destroy()

    def update_data(self):
        """Refreshing robot data while main thread is working"""
        temp = 0
        incr = 1
        while threading.main_thread().is_alive():

            data = self.model.get_data()
            lat = data['GPS'].latitude
            long = data['GPS'].longitude
            pose = data['Odom'].pose
            velocity = data['Odom'].twist
            diagnostic = data['Diagnostic']
            # orientation = data['Odom'].pose.orientation
            if type(lat) is float and type(long) is float:
                self.gps_data['Lat'].set(round(lat, 10))
                self.gps_data['Long'].set(round(long, 10))
                #self.robot_marker.set_position(lat, long)
                #self.robot_path.add_position(self.robot_marker.position[0], self.robot_marker.position[1])
                #self.robot_path.draw()
            if not isinstance(pose, types.MemberDescriptorType):
                position = pose.pose.position
                orientation = pose.pose.orientation
                self.odom_data['Position'][0].set(round(position.x, 3))
                self.odom_data['Position'][1].set(round(position.y, 3))
                self.odom_data['Position'][2].set(round(position.z, 3))

                self.odom_data['Orientation'][0].set(round(orientation.x, 3))
                self.odom_data['Orientation'][1].set(round(orientation.y, 3))
                self.odom_data['Orientation'][2].set(round(orientation.z, 3))
                self.odom_data['Orientation'][3].set(round(orientation.w, 3))
            if not isinstance(velocity, types.MemberDescriptorType):
                linear = velocity.twist.linear
                angular = velocity.twist.angular
                self.odom_data['LinearVel'][0].set(round(linear.x, 3))
                self.odom_data['LinearVel'][1].set(round(linear.y, 3))
                self.odom_data['LinearVel'][2].set(round(linear.z, 3))

                self.odom_data['AngularVel'][0].set(round(angular.x, 3))
                self.odom_data['AngularVel'][1].set(round(angular.y, 3))
                self.odom_data['AngularVel'][2].set(round(angular.z, 3))
            # print(diagnostic)
            if not isinstance(diagnostic, type(HuskyStatus)):
                diagnostic = diagnostic.status
                # print(round(float(diagnostic[0].values[7].value), 3))
                self.diagnostic_data['Temperature']['LeftDriver'].set(round(float(diagnostic[0].values[7].value), 3))
                self.diagnostic_data['Temperature']['LeftMotor'].set(round(float(diagnostic[0].values[9].value), 3))
                self.diagnostic_data['Temperature']['RightDriver'].set(round(float(diagnostic[0].values[8].value), 3))
                self.diagnostic_data['Temperature']['RightMotor'].set(round(float(diagnostic[0].values[10].value), 3))
                # print(diagnostic[1].values[0])
                self.diagnostic_data['Battery']['Capacity'].set(round(float(diagnostic[1].values[1].value), 3))
                self.diagnostic_data['Battery']['Charge'].set(round(100 * float(diagnostic[1].values[0].value), 3))
                self.update_battery_bar()
                self.diagnostic_data['ErrorsStatus']['Timeout'].set(diagnostic[2].values[0].value)
                self.diagnostic_data['ErrorsStatus']['Lockout'].set(diagnostic[2].values[1].value)
                self.diagnostic_data['ErrorsStatus']['Estop'].set(diagnostic[2].values[2].value)
                self.diagnostic_data['ErrorsStatus']['RosPause'].set(diagnostic[2].values[3].value)
                self.diagnostic_data['ErrorsStatus']['NoBattery'].set(diagnostic[2].values[4].value)
                self.diagnostic_data['ErrorsStatus']['CurrentLimit'].set(diagnostic[2].values[5].value)
            # if len(self.action_list) > 0:
            #    print(self.action_list[0].get_data()['RotateBy']['direction'].get())
            rospy.sleep(1 / 60)
            # time.sleep(1 / 3)

    def executor(self):
        t = Thread(name="executor", target=self.execute)
        t.start()

    def deployer(self):
        t = Thread(name="deployer", target=self.deploy_aidkit)
        t.start()

    def execute(self):
        for action in self.action_list:
            action.execute()

    def remove_action(self, action):
        self.action_list.remove(action)
        for it, action in enumerate(self.action_list):
            action.grid(row=0, column=it)

    def add_action_on_click(self):
        self.action_popup_window.deiconify()

    def add_action(self, action_number):
        print(action_number)
        self.action_list.append(
            ActionModel(self.action_list_frame, action_number, 0, len(self.action_list), self.model, self))

    def target_walker(self):
        pass

    def create_layout(self):
        """Packs basic frames to root frame"""
        self.top_left_frame = ttk.Frame(self.root, padding=5)
        self.top_center_frame = ttk.Frame(self.root, padding=5)
        self.top_right_frame = ttk.Frame(self.root, padding=5)
        self.bottom_frame = ttk.Frame(self.root, padding=5)

        self.top_left_frame.grid(row=0, column=0, sticky=NS)  # pack(side=LEFT, anchor="nw",fill='both', expand=False)
        self.top_center_frame.grid(row=0, column=1, sticky=NS)  # pack(side=LEFT,anchor="nw", fill='both', expand=False)
        self.top_right_frame.grid(row=0, column=2, sticky=NS)  # pack(side=LEFT, anchor="nw",fill='both', expand=False)
        self.bottom_frame.grid(row=1, columnspan=3, column=0,
                               sticky=NSEW)  # ack(side=BOTTOM, anchor="s", fill='both', expand=False)

        self.menu_label_frame = ttk.LabelFrame(self.top_left_frame, text='Menu', padding=50)
        self.menu_label_frame.pack(expand=False, anchor="n")

        self.main_status_frame = ttk.LabelFrame(self.top_center_frame, text='Current Robot States & Actions',
                                                padding=50)
        self.main_status_frame.pack(fill='both', expand=True)

    def save_action(self):
        tz = pytz.timezone("Europe/Warsaw")
        ct = datetime.datetime.now(tz)
        #ct = datetime.datetime.now(datetime.timezone.utc)
        self.mission_log_data += f"{self.curr_action_var.get()} - {ct}\n"
        self.kml_mission_logger.add_waypoint_event(self.gps_data['Lat'].get(), self.gps_data['Long'].get(), self.curr_action_var.get(), ct)

    def create_mission_components(self):
        self.curr_action_var = tk.StringVar()
        current_action_frame = ttk.LabelFrame(self.top_left_frame, text='Current Action', padding=20)
        current_action_frame.pack(anchor='n', fill='both', expand=True)

        MSD_frame = ttk.LabelFrame(current_action_frame, text='MSD action features', padding=50)
        MSD_frame.pack(anchor='n', fill='both', expand=True)

        ORI_frame = ttk.LabelFrame(current_action_frame, text='ORI action features', padding=50)
        ORI_frame.pack(anchor='n', fill='both', expand=True)

        WAYPOINT_frame = ttk.LabelFrame(current_action_frame, text='WAYPOINT action features', padding=50)
        WAYPOINT_frame.pack(anchor='n', fill='both', expand=True)


        # MSD SECTION #
        combo = ttk.Combobox(MSD_frame, state='readonly', values=self.event_list,
                             textvariable=self.curr_action_var)
        combo.set(self.event_list[0])
        combo.pack()
        # ttk.Entry(current_action_frame, textvariable=self.curr_action_var, width=10).pack()
        ttk.Button(MSD_frame, text='Save Action', command=self.save_action).pack(fill='x', expand=True)

        #MSD SECTION #


        # ORI SECTION #
        self.curr_camera_var = tk.StringVar()
        self.image_name = tk.StringVar()
        self.target_id = tk.StringVar()
        ttk.Label(ORI_frame, text="Chose camera").pack()

        combo = ttk.Combobox(ORI_frame, state='readonly', values=self.camera_list,
                             textvariable=self.curr_camera_var)
        combo.set(self.camera_list[0])
        combo.pack(pady=10)
        ttk.Label(ORI_frame, text="Enter image name").pack()

        ttk.Entry(ORI_frame, textvariable= self.image_name).pack()
        ttk.Label(ORI_frame, text="Enter target id").pack()
        ttk.Entry(ORI_frame, textvariable= self.target_id).pack()
        ttk.Button(ORI_frame, text='Capture image', command=self.save_image).pack(fill='x', expand=True, pady=5)
        ttk.Button(ORI_frame, text='Save target', command=self.save_target).pack(fill='x', expand=True, pady=5)

        # ORI SECTION #

        # WAYPOINT SECTION #
        self.waypoint_long = tk.StringVar()
        self.waypoint_lat = tk.StringVar()
        ttk.Label(WAYPOINT_frame, text="Waypoint name").pack()
        ttk.Entry(WAYPOINT_frame, textvariable=self.waypoint_name_var).pack()
        ttk.Label(WAYPOINT_frame, text="Latitude").pack()
        ttk.Entry(WAYPOINT_frame, textvariable=self.waypoint_lat).pack()
        ttk.Label(WAYPOINT_frame, text="Longitude").pack()
        ttk.Entry(WAYPOINT_frame, textvariable=self.waypoint_long).pack()
        ttk.Button(WAYPOINT_frame, text='Add waypoint', command=self.add_waypoint).pack(fill='x', expand=True, pady=5)
        #WAYPOINT SECTION

        self.action_list_frame = ttk.LabelFrame(self.bottom_frame, text='Action list', labelanchor="n", padding=20)
        self.action_list_frame.pack(anchor='n', fill='both', expand=True)

    # def create_map_components(self):
    #     map_frame = ttk.LabelFrame(self.top_right_frame, text='Map', padding=20)
    #     map_frame.pack(anchor="nw", fill="x")
    #     self.map_widget = TkinterMapView(map_frame, width=1200, height=1000, corner_radius=0)
    #     self.map_widget.pack(fill="both", expand=True)
    #     self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
    #     self.map_widget.set_address("Kąkolewo, Poland", marker=False)
    #     self.map_widget.set_position(52.2366, 16.2446)
    #     robot_image = tk.PhotoImage(file='utilities/robot_img1.png')
    #     self.robot_marker = self.map_widget.set_marker(52.2366, 16.2446, text="Husky", image=robot_image)
    #
    #     self.robot_marker.image_zoom_visibility = (0, float('inf'))
    #     self.robot_path = self.map_widget.set_path(
    #         [self.robot_marker.position, self.robot_marker.position, self.robot_marker.position,
    #          self.robot_marker.position])

    def create_data_variables(self):
        self.gps_data = {"Lat": ttk.StringVar(value='0.0'),
                         "Long": ttk.StringVar(value='0.0')}
        self.odom_data = {"Position":
                              [ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0')],
                          "Orientation":
                              [ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0')],
                          "LinearVel":
                              [ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0')],
                          "AngularVel":
                              [ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0'),
                               ttk.StringVar(value='0.0')]
                          }
        self.diagnostic_data = {"Temperature":
                                    {"LeftDriver": ttk.StringVar(value='0.0'),
                                     "LeftMotor": ttk.StringVar(value='0.0'),
                                     "RightDriver": ttk.StringVar(value='0.0'),
                                     "RightMotor": ttk.StringVar(value='0.0')
                                     },
                                "Battery":
                                    {"Capacity": ttk.StringVar(value='0.0'),
                                     "Charge": ttk.StringVar(value='0.0')
                                     },
                                "ErrorsStatus":
                                    {"Timeout": ttk.StringVar(value='0.0'),
                                     "Lockout": ttk.StringVar(value='0.0'),
                                     "Estop": ttk.StringVar(value='0.0'),
                                     "RosPause": ttk.StringVar(value='0.0'),
                                     "NoBattery": ttk.StringVar(value='0.0'),
                                     "CurrentLimit": ttk.StringVar(value='0.0')
                                     }
                                }

    def create_menu_components(self):
        # Popup window for choosing action from list #
        self.action_popup_window = tk.Toplevel(self.menu_label_frame)
        self.action_popup_window.title(f"Choose action")
        self.action_popup_window.protocol("WM_DELETE_WINDOW", self.action_popup_window.withdraw)
        self.action_popup_window.withdraw()
        tk.Button(self.action_popup_window, text=ActionType.MoveTo.name,
                  command=lambda: self.add_action(ActionType.MoveTo)).pack(fill='both')
        tk.Button(self.action_popup_window, text=ActionType.DeployAidKit.name,
                  command=lambda: self.add_action(ActionType.DeployAidKit)).pack(fill='both')
        tk.Button(self.action_popup_window, text=ActionType.Explore.name,
                  command=lambda: self.add_action(ActionType.Explore)).pack(fill='both')
        tk.Button(self.action_popup_window, text=ActionType.SeekForHuman.name,
                  command=lambda: self.add_action(ActionType.SeekForHuman)).pack(fill='both')
        tk.Button(self.action_popup_window, text=ActionType.RotateBy.name,
                  command=lambda: self.add_action(ActionType.RotateBy)).pack(fill='both')
        tk.Button(self.action_popup_window, text=ActionType.MoveBy.name,
                  command=lambda: self.add_action(ActionType.MoveBy)).pack(fill='both')
        tk.Button(self.action_popup_window, text=ActionType.MoveThrough.name,
                  command=lambda: self.add_action(ActionType.MoveThrough)).pack(fill='both')

        # # # # # # # # # # # # # # # # # # # # # # # #
        ttk.Button(self.menu_label_frame, text='Add Action', width=10, command=self.add_action_on_click).pack(pady=5)
        self.marker_entry = ttk.Entry(self.menu_label_frame, width=10)
        self.marker_entry.pack(pady=5)
        ttk.Button(self.menu_label_frame, text='Deploy Aidkit', width=10, command=self.deployer).pack(pady=5)
        # ttk.Button(self.menu_label_frame, text='Load markers', width=10, command=self.load_waypoints).pack(pady=5)
        # ttk.Button(self.menu_label_frame, text='Remove markers', width=10, command=self.remove_waypoint).pack(pady=5)
        ttk.Button(self.menu_label_frame, text='Execute', width=10, command=self.executor).pack(pady=5)
        ttk.Button(self.menu_label_frame, text='Abort', width=10, command=self.target_walker).pack(pady=5)
        self.mission_progress_viz = ttk.Meter(self.menu_label_frame, metersize=180,
                                              amountused=0,
                                              padding=15,
                                              textright=" %",
                                              subtextfont="-size 10 -weight bold",
                                              metertype="semi",
                                              subtext="Mission",
                                              interactive=True,
                                              stripethickness=10,
                                              )
        self.mission_progress_viz.pack()

    def deploy_aidkit(self):
        # rospy.wait_for_service('send_command')
        try:
            send_command = rospy.ServiceProxy('send_command', Launcher)
            command_response = send_command(1)
            return command_response.response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def create_status_components(self):
        ### GPS status components ###
        gps_status_frame = ttk.LabelFrame(self.main_status_frame, text='GPS status', padding=20)
        gps_status_frame.pack(anchor="nw", fill="x")
        # Latitude #
        ttk.Label(gps_status_frame, text="Latitude: ").pack(side=LEFT)
        ttk.Label(gps_status_frame, textvariable=self.gps_data['Lat'], width=20).pack(side=LEFT)
        # Longitude #
        ttk.Label(gps_status_frame, text="Longitude: ").pack(side=LEFT)
        ttk.Label(gps_status_frame, textvariable=self.gps_data['Long'], width=20).pack(side=LEFT)
        ### Odometry status components ###
        odometry_status_frame = ttk.LabelFrame(self.main_status_frame, text='Odometry status', padding=20)
        odometry_status_frame.pack(side=TOP, fill="x", pady=15)
        # Position #
        ttk.Label(odometry_status_frame, text="Position: ", width=15).grid(column=0, row=0)
        ttk.Label(odometry_status_frame, text="X: ").grid(column=1, row=0)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['Position'][0], width=7).grid(column=2, row=0)
        ttk.Label(odometry_status_frame, text="Y: ").grid(column=3, row=0)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['Position'][1], width=7).grid(column=4, row=0)
        ttk.Label(odometry_status_frame, text="Z: ").grid(column=5, row=0)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['Position'][2], width=7).grid(column=6, row=0)
        # Orientation #
        ttk.Label(odometry_status_frame, text="Orientation: ", width=15).grid(column=0, row=1, pady=15)
        ttk.Label(odometry_status_frame, text="X: ").grid(column=1, row=1)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['Orientation'][0], width=7).grid(column=2, row=1)
        ttk.Label(odometry_status_frame, text="Y: ").grid(column=3, row=1)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['Orientation'][1], width=7).grid(column=4, row=1)
        ttk.Label(odometry_status_frame, text="Z: ").grid(column=5, row=1)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['Orientation'][2], width=7).grid(column=6, row=1)
        ttk.Label(odometry_status_frame, text="W: ").grid(column=7, row=1)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['Orientation'][3], width=7).grid(column=8, row=1)
        # Linear velocity #
        ttk.Label(odometry_status_frame, text="Linear Velocity: ", width=15).grid(column=0, row=2, pady=15)
        ttk.Label(odometry_status_frame, text="X: ").grid(column=1, row=2)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['LinearVel'][0], width=7).grid(column=2, row=2)
        ttk.Label(odometry_status_frame, text="Y: ").grid(column=3, row=2)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['LinearVel'][1], width=7).grid(column=4, row=2)
        ttk.Label(odometry_status_frame, text="Z: ").grid(column=5, row=2)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['LinearVel'][2], width=7).grid(column=6, row=2)
        # Angular velocity #
        ttk.Label(odometry_status_frame, text="Angular Velocity: ", width=15).grid(column=0, row=3, pady=15)
        ttk.Label(odometry_status_frame, text="X: ").grid(column=1, row=3)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['AngularVel'][0], width=7).grid(column=2, row=3)
        ttk.Label(odometry_status_frame, text="Y: ").grid(column=3, row=3)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['AngularVel'][1], width=7).grid(column=4, row=3)
        ttk.Label(odometry_status_frame, text="Z: ").grid(column=5, row=3)
        ttk.Label(odometry_status_frame, textvariable=self.odom_data['AngularVel'][2], width=7).grid(column=6, row=3)
        ### Robot diagnostic components ###
        diagnostic_status_frame = ttk.LabelFrame(self.main_status_frame, text='Diagnostic robot status', padding=20)
        diagnostic_status_frame.pack(side=TOP, fill="x")
        # Component temperatures #
        temperature_frame = ttk.LabelFrame(diagnostic_status_frame, text="Component temperatures [℃]")
        temperature_frame.pack(side=TOP, fill="x", pady=15)

        ttk.Label(temperature_frame, text="Left driver: ").grid(column=0, row=0, pady=15, padx=15)
        ttk.Label(temperature_frame,
                  textvariable=self.diagnostic_data['Temperature']['LeftDriver'], width=7).grid(column=1, row=0)
        ttk.Label(temperature_frame, text="Left motor: ").grid(column=2, row=0)
        ttk.Label(temperature_frame,
                  textvariable=self.diagnostic_data['Temperature']['LeftMotor'], width=7).grid(column=3, row=0)
        ttk.Label(temperature_frame, text="Right driver: ").grid(column=4, row=0)
        ttk.Label(temperature_frame,
                  textvariable=self.diagnostic_data['Temperature']['RightDriver'], width=7).grid(column=5, row=0)
        ttk.Label(temperature_frame, text="Right motor: ").grid(column=6, row=0)
        ttk.Label(temperature_frame,
                  textvariable=self.diagnostic_data['Temperature']['RightMotor'], width=7).grid(column=7, row=0)
        # Battery #
        battery_frame = ttk.LabelFrame(diagnostic_status_frame, text="Battery status")
        battery_frame.pack(side=TOP, fill="x", pady=15)

        ttk.Label(battery_frame, text="Capacity estimated [Wh]:").grid(column=0, row=0, pady=15, padx=15)
        ttk.Label(battery_frame,
                  textvariable=self.diagnostic_data['Battery']['Capacity'], width=7).grid(column=1, row=0)
        ttk.Label(battery_frame, text="Charge estimated [%]:").grid(column=2, row=0, padx=15)
        ttk.Label(battery_frame,
                  textvariable=self.diagnostic_data['Battery']['Charge'], width=7).grid(column=3, row=0)
        self.create_battery_viz(battery_frame, self.diagnostic_data['Battery']['Charge'], 4, 0)
        # Error Status #
        error_frame = ttk.LabelFrame(diagnostic_status_frame, text="Error status")
        error_frame.pack(side=TOP, fill="x", pady=15)

        ttk.Label(error_frame, text="Timeout: ").grid(column=0, row=0, pady=15, padx=15)
        ttk.Label(error_frame,
                  textvariable=self.diagnostic_data['ErrorsStatus']['Timeout'], width=7).grid(column=1, row=0)
        ttk.Label(error_frame, text="Lockout: ").grid(column=2, row=0)
        ttk.Label(error_frame,
                  textvariable=self.diagnostic_data['ErrorsStatus']['Lockout'], width=7).grid(column=3, row=0)
        ttk.Label(error_frame, text="Estop: ").grid(column=4, row=0)
        ttk.Label(error_frame,
                  textvariable=self.diagnostic_data['ErrorsStatus']['Estop'], width=7).grid(column=5, row=0)
        ttk.Label(error_frame, text="RosPause:").grid(column=6, row=0)
        ttk.Label(error_frame,
                  textvariable=self.diagnostic_data['ErrorsStatus']['RosPause'], width=7).grid(column=7, row=0)
        ttk.Label(error_frame, text="NoBattery:").grid(column=8, row=0)
        ttk.Label(error_frame,
                  textvariable=self.diagnostic_data['ErrorsStatus']['NoBattery'], width=7).grid(column=9, row=0)
        ttk.Label(error_frame, text="CurrentLimit:").grid(column=10, row=0)
        ttk.Label(error_frame,
                  textvariable=self.diagnostic_data['ErrorsStatus']['CurrentLimit'], width=7).grid(column=11, row=0)

    def create_battery_viz(self, parent, variable, column, row):
        TROUGH_COLOR = "#000000"
        BAR_COLOR = 'green'

        self.root.style.configure("bar.Horizontal.TProgressbar", troughcolor=TROUGH_COLOR,
                                  bordercolor=BAR_COLOR, background=BAR_COLOR, lightcolor=BAR_COLOR,
                                  darkcolor=BAR_COLOR, thickness=30)
        ttk.Progressbar(parent, orient=HORIZONTAL, length=100, mode='determinate', variable=variable,
                        style="bar.Horizontal.TProgressbar").grid(column=column, row=row, pady=15)

    def update_battery_bar(self):
        """
        Update battery visualization bar based on current estimated charge level
        Red  <= 25%
        Yellow 25% - 75%
        Green >= 75%
        """
        curr_val = float(self.diagnostic_data['Battery']['Charge'].get())
        if 25.0 < curr_val < 50.0:
            BAR_COLOR = 'yellow'
            self.root.style.configure("bar.Horizontal.TProgressbar",
                                      bordercolor=BAR_COLOR, background=BAR_COLOR, lightcolor=BAR_COLOR,

                                      darkcolor=BAR_COLOR)
        elif curr_val <= 25.0:
            BAR_COLOR = 'red'

            self.root.style.configure("bar.Horizontal.TProgressbar",
                                      bordercolor=BAR_COLOR, background=BAR_COLOR, lightcolor=BAR_COLOR,
                                      darkcolor=BAR_COLOR)
        else:
            BAR_COLOR = 'green'

            self.root.style.configure("bar.Horizontal.TProgressbar",
                                      bordercolor=BAR_COLOR, background=BAR_COLOR, lightcolor=BAR_COLOR,
                                      darkcolor=BAR_COLOR)

    # def load_waypoints(self):
    #     f = open(f"utilities/marker_coords.json")
    #     data = json.load(f)
    #
    #     for i, marker in enumerate(data):
    #         self.marker_list[f'marker_{i}'] = self.map_widget.set_marker(marker[0], marker[1], text=f'Marker_{i}')
    #
    # def remove_waypoint(self):
    #     temp_string = self.marker_entry.get()
    #     self.marker_list[f'marker_{temp_string}'].delete()
    def add_waypoint(self):
        # self.kml_generator.add_waypoint(self.gps_data['Lat'], self.gps_data['Long'],self.waypoint_name_var.get())
        self.kml_waypoint_logger.add_waypoint(self.waypoint_lat.get(), self.waypoint_long.get(), self.waypoint_name_var.get())

    def save_image(self):
        #ct = datetime.datetime.now()
        #ct = ct.strftime("%d_%m_%Y_%H:%M:%S")
        bridge = CvBridge()
        image_message = rospy.wait_for_message(f"/usb_cam_{self.curr_camera_var.get()}/image_raw", sensors.Image)
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        pixels = np.array(cv_image)
        image = Image.fromarray(pixels.astype('uint8'), 'RGB')
        image.save(f'captured_images/{self.image_name.get()}.jpg')
