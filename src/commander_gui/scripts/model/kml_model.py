import os
import signal
import sys
import math
import threading
import time
import string
import rospy
import roslib;
from zipfile import ZipFile

roslib.load_manifest('kml_generator')
import rosbag
import tf
from nav_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from pykml.factory import nsmap
from pykml.factory import KML_ElementMaker as KML
from pykml.factory import GX_ElementMaker as GX
from pykml.parser import Schema
from lxml import etree
import simplekml
import utm


class KmlModel(simplekml.Kml):
    def __init__(self, name, indoor_status):
        super().__init__()
        self.indoor_status = indoor_status
        self.name = name
        self.point_coords_list = []
        self.images_list = []
        self.VND_path = self.newdocument(name="UVG path")
        self.VND_path.style.labelstyle.scale = 0
        #self.VND_path.style.iconstyle.scale = 0
        self.MSD_folder = self.newdocument(name="Mission Status Data")
        self.MI_folder = self.newdocument(name="Map information")
        self.ORI_folder = self.newdocument(name="Object recognition information")
        self.WPD_folder = self.newdocument(name="Waypoint Data")

    def add_path_point(self, lat, long, heading, utc_time):
        print("dodalem")
        pnt = self.VND_path.newpoint(coords=[(long, lat)])
        pnt.description =f"Lat: {lat}\nLong: {long}\nHeading: {heading}\nTime: {utc_time}"
        pnt.timestamp.when = utc_time
        pnt.style.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle_highlight.png'
        self.point_coords_list.append((long, lat))

    def add_waypoint(self, name, lat, long):
        waypoint = self.WPD_folder.newpoint(name=name, coords=[(long, lat)])

    def add_ori(self, name, lat, long, image_name=None, image_full_path=None):
        ori = self.ORI_folder.newpoint(name=name, coords=[(long, lat)])
        if image_full_path is not None:
            self.images_list.append(image_full_path)
            ori.description = '<img src=\"files/' + image_name + '\" alt=\"picture\" width=\"300\" height=\"250\" align=\"left\" />'
            ori.style.iconstyle.icon.href = f"captured_images/{image_name}.jpg"

    def add_mission_status_data(self, name, desc, utc_time):
        event = self.MSD_folder.newpoint(name=name, coords=[(16.2308,52.238611)])
        event.description = desc #+ f"Event time: {utc_time}"
        event.timestamp.when = utc_time
        #event.timestamp.when = utc_time

    def finish(self, path):
        temp = self.newlinestring()
        temp.coords = self.point_coords_list
        temp.extrude = 1
        self.savekmz(path)


class KMLTourGenerator(object):

    # ---------------------------------------------------------------------------
    def __init__(self, name):
        rospy.loginfo("[KMLTourGenerator] init()")

        self.hasFinished = False

        self.utmZoneNumber = 33
        self.utmZoneLetter = 'U'
        self.zOffset = 2.0

        self.coord_list = []
        self.images_list = []

        self.planCoordListStr = ''
        self.execCoordListStr = ''

        self.path_fld = KML.Folder(
            KML.name(f'{name}'),
            id='waypoints',
        )

    def finish(self, kmlFilename):
        # rospy.loginfo("[KMLTourGenerator] finish()")
        # outfile = open(kmlFilename, 'wb')
        # outfile.write(etree.tostring(self.tour_doc, pretty_print=True))
        # outfile.close()
        # self.hasFinished = True
        with ZipFile(f'{kmlFilename}.kmz', 'w') as zipObj:
            zipObj.writestr(f'{kmlFilename}.kml', etree.tostring(self.path_fld, pretty_print=True, xml_declaration=True,
                                                                 encoding='UTF-8'))  # Add doc.kml entry
            for image in self.images_list:
                zipObj.write(image)  # Add icon to the zip

    # ---------------------------------------------------------------------------
    def add_waypoint_wth_image(self, lat, long, name, img_full_path):
        # pos[0] : latitude
        # pos[1] : longitude
        # pos[2] : altitude
        # pos[3] : roll
        # pos[4] : pitch
        # pos[5] : yaw
        self.images_list.append(img_full_path)
        self.path_fld.append(KML.Placemark(
            KML.name(name),
            KML.Style(
                KML.IconStyle(
                    KML.scale(1.0),
                    KML.Icon(
                        KML.href(img_full_path)
                    )
                )
            ),
            KML.Point(
                KML.tessellate("1"),
                KML.altitudeMode("Clamped to Ground"),
                KML.coordinates(f"{long}, {lat}")
            )
        ))

    def add_waypoint(self, lat, long, name):
        # pos[0] : latitude
        # pos[1] : longitude
        # pos[2] : altitude
        # pos[3] : roll
        # pos[4] : pitch
        # pos[5] : yaw
        self.path_fld.append(KML.Placemark(
            KML.name(name),
            KML.Point(
                KML.tessellate("1"),
                KML.altitudeMode("Clamped to Ground"),
                KML.coordinates(f"{long}, {lat}")
            )
        ))

    def add_waypoint_event(self, lat, long, event_name, event_occurs_time):
        self.path_fld.append(KML.Placemark(
            KML.name(event_name),
            KML.Point(
                KML.tessellate("1"),
                KML.altitudeMode("Clamped to Ground"),
                KML.coordinates(f"{long}, {lat}")
            ),
            KML.ExtendedData(KML.Data(KML.value(event_occurs_time), name="Event time")),
        ))
