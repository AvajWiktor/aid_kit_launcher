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
            zipObj.write('utilities/0.png')  # Add icon to the zip

    # ---------------------------------------------------------------------------
    def add_waypoint(self, lat, long, name):
        # pos[0] : latitude
        # pos[1] : longitude
        # pos[2] : altitude
        # pos[3] : roll
        # pos[4] : pitch
        # pos[5] : yaw
        self.path_fld.append(KML.Placemark(
            KML.name(name),
            KML.Style(
                KML.IconStyle(
                    KML.scale(1.0),
                    KML.Icon(
                        KML.href("utilities/0.png")
                    )
                )
            ),
            KML.Point(
                KML.tessellate("1"),
                KML.altitudeMode("absolute"),
                KML.coordinates(f"{long}, {lat}")
            )
        ))
        # auxStr = ' {lon},{lat},{alt}\n'.format(lat=lat, lon=long, alt=0.0)
        # self.execCoordListStr = self.execCoordListStr + auxStr
