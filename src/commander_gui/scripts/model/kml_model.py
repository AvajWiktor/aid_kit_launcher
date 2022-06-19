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

import utm


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
            zipObj.writestr(f'{kmlFilename}.kml', etree.tostring(self.path_fld, pretty_print=True, xml_declaration=True, encoding='UTF-8'))  # Add doc.kml entry
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

    def add_waypoint_event(self,lat, long, event_name, event_occurs_time):
        self.path_fld.append(KML.Placemark(
            KML.name(event_name),
            KML.Point(
                KML.tessellate("1"),
                KML.altitudeMode("Clamped to Ground"),
                KML.coordinates(f"{long}, {lat}")
            ),
            KML.ExtendedData(KML.Data(KML.value(event_occurs_time), name="Event time")),
        ))
