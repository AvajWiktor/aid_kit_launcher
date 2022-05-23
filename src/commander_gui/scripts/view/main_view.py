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
from geometry_msgs.msg import Pose, Point, Quaternion
from tkinter import filedialog as fd
from tkintermapview import TkinterMapView
import time
import json


class MainWindowView:
    def __init__(self, robot_model):
        self.root = ttk.Window(themename='cyborg', title='Commander GUI')
        self.root.minsize(width=1920, height=800)
        self.robot_model = robot_model
        self.model = MainModel(self.robot_model)
        self.controller = MainController(self.robot_model, self.model)
        self.updater = Thread(name='refresher', target=self.update_data)
        self.tag_id = ttk.StringVar(value='0')
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
        """
        *Code here*
        """
        self.create_layout()
        self.add_menu_components()
        self.add_status_components()
        self.create_map_features()
        """
        *Code End*
        """
        self.updater.start()
        self.root.mainloop()

    def update_data(self):
        """Refreshing robot data while main thread is working"""
        while threading.main_thread().is_alive():
            data = self.model.get_data()
            lat = data['GPS'].latitude
            long = data['GPS'].longitude
            pose = data['Odom'].pose
            velocity = data['Odom'].twist
            # orientation = data['Odom'].pose.orientation
            if type(lat) is float and type(long) is float:
                self.gps_data['Lat'].set(round(lat, 10))
                self.gps_data['Long'].set(round(long, 10))
                self.robot_marker.set_position(lat, long)
                self.robot_path.add_position(self.robot_marker.position[0], self.robot_marker.position[1])
                self.robot_path.draw()
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

            time.sleep(1 / 3)

    def executor(self):
        pass

    def walker(self):
        pass

    def target_walker(self):
        pass

    def connector(self):
        connector = Thread(name='Connector', target=self.controller.connect_to_robot)
        # self.controller.connect_to_robot()
        connector.start()

    def create_layout(self):
        """Packs basic frames to root frame"""
        self.left_frame = ttk.Frame(self.root, padding=(5, 5, 5, 5))
        self.center_frame = ttk.Frame(self.root, padding=(5, 5, 5, 5))
        self.right_frame = ttk.Frame(self.root, padding=5)

        self.left_frame.pack(side=LEFT, fill='both', expand=False)
        self.center_frame.pack(side=LEFT, fill='both', expand=True)
        self.right_frame.pack(side=LEFT, fill='both', expand=True)

        self.menu_label_frame = ttk.LabelFrame(self.left_frame, text='Menu', padding=50)
        self.menu_label_frame.pack(fill='both')

        self.main_status_frame = ttk.LabelFrame(self.center_frame, text='Current Robot States', padding=50)
        self.main_status_frame.pack(fill='both', expand=True)

    def create_map_features(self):
        map_widget = TkinterMapView(self.right_frame, width=1500, height=1500, corner_radius=0)
        map_widget.pack(fill="both", expand=True)
        map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        map_widget.set_address("Kąkolewo, Poland", marker=False)
        robot_image = tk.PhotoImage(file='utilities/robot_img.png')
        self.robot_marker = map_widget.set_marker(52.2366, 16.2446, text="Husky", image=robot_image)

        self.robot_marker.image_zoom_visibility = (0, float('inf'))
        self.robot_path = map_widget.set_path([self.robot_marker.position, self.robot_marker.position, self.robot_marker.position, self.robot_marker.position])

    def add_menu_components(self):
        ttk.Label(self.menu_label_frame, text='Tag id').pack()
        ttk.Button(self.menu_label_frame, text='Connect', width=10, command=self.connector).pack(pady=5)
        ttk.Button(self.menu_label_frame, text='Go to tag', width=10, command=self.walker).pack(pady=5)
        ttk.Button(self.menu_label_frame, text='Go to target', width=10, command=self.target_walker).pack(pady=5)
        ttk.Button(self.menu_label_frame, text='Execute', width=10, command=self.executor).pack(pady=5)

    def add_status_components(self):
        ### GPS status components ###
        gps_status_frame = ttk.LabelFrame(self.main_status_frame, text='GPS status', padding=20)
        gps_status_frame.pack(anchor="nw", fill="x", expand=True)
        # Latitude #
        ttk.Label(gps_status_frame, text="Latitude: ").pack(side=LEFT)
        ttk.Label(gps_status_frame, textvariable=self.gps_data['Lat'], width=20).pack(side=LEFT)
        # Longitude #
        ttk.Label(gps_status_frame, text="Longitude: ").pack(side=LEFT)
        ttk.Label(gps_status_frame, textvariable=self.gps_data['Long'], width=20).pack()
        ### Odometry status components ###
        odometry_status_frame = ttk.LabelFrame(self.main_status_frame, text='Odometry status', padding=20)
        odometry_status_frame.pack(anchor="nw", fill="x", expand=True)
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

    def add_image(self, img_name):
        img = Image.open(f'utilities/{img_name}.png')
        img.resize((50, 50), Image.ANTIALIAS)
        test = ImageTk.PhotoImage(img)
        img_label = tk.Label(self.main_status_frame, image=test)
        img_label.image = test

        # Position image
        img_label.pack()  # place(x= 10, y = 10)
        # Create a Label Widget to display the text or Image