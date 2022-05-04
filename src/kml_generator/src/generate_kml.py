#!/usr/bin/env python3
import os
import signal
import sys
import math
import threading
import time
import string
import rospy
import roslib;

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


# -------------------------------------------------------------------------------
# GLOBAL VARS


# ===============================================================================

class KMLTourGenerator(object):

    # ---------------------------------------------------------------------------
    def __init__(self, poseTopicName, orientTopicName, utmZoneNumber):
        rospy.loginfo("[KMLTourGenerator] init()")

        self.hasFinished = False

        self.hasOdom = False
        self.hasOrient = False

        self.currOrient = None
        self.prevOrient = None

        self.currOdom = None
        self.prevOdom = None
        self.subPosition = rospy.Subscriber(poseTopicName, NavSatFix, self.poseHandler)
        self.subOrient = rospy.Subscriber(orientTopicName, Odometry, self.orientHandler)

        self.utmZoneNumber = utmZoneNumber
        self.utmZoneLetter = 'U'
        self.zOffset = 2.0

        self.planCoordListStr = ''
        self.execCoordListStr = ''

        self.totalTime = 0.0

        # define a variable for the Google Extensions namespace URL string
        self.gxns = '{' + nsmap['gx'] + '}'

        # start with a base KML tour and playlist

        self.anim_fld = KML.Folder(
            KML.name('Animations'),
            id='animations',
        )

        self.path_fld = KML.Folder(
            KML.name('Paths'),
            id='paths',
        )

        self.tour_doc = KML.kml(
            KML.Document(
                KML.Style(
                    KML.IconStyle(
                        KML.scale(1.2),
                        KML.Icon(
                            KML.href("http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png")
                        ),
                    ),
                    id='sn_shaded_dot',
                ),
                GX.Tour(
                    KML.name("Play me!"),
                    GX.Playlist(),
                ),
                self.anim_fld,
                self.path_fld
            )
        )

    # ---------------------------------------------------------------------------
    # use odometer information to compute some metrics
    def poseHandler(self, odom):
        print('pose handler')
        self.hasOdom = True
        if (self.prevOdom == None):
            self.prevOdom = odom
        self.currOdom = odom

    def orientHandler(self, orient):
        print('orient handler')
        self.hasOrient = True
        if (self.prevOrient == None):
            self.prevOrient = orient
        self.currOrient = orient

    # ---------------------------------------------------------------------------
    def finish(self, kmlFilename):
        rospy.loginfo("[KMLTourGenerator] finish()")
        if (self.hasFinished):
            return

        self.path_fld.append(KML.Placemark(
            KML.name('plan-path'),
            KML.extrude("1"),
            KML.tessellate("1"),
            KML.altitudeMode("absolute"),
            KML.Style(
                KML.LineStyle(
                    KML.color('7FFF0000'),
                    KML.width(5)
                ),
                KML.PolyStyle(
                    KML.color('7FFFFF00')
                )
            ),
            KML.LineString(
                KML.tessellate("1"),
                KML.altitudeMode("absolute"),
                KML.coordinates(self.planCoordListStr)
            )
        ))

        self.path_fld.append(KML.Placemark(
            KML.name('exec-path'),
            KML.altitudeMode("absolute"),
            KML.Style(
                KML.LineStyle(
                    KML.color('7F0000FF'),
                    KML.width(5)
                ),
                KML.PolyStyle(
                    KML.color('7FFFFFFF')
                )
            ),
            KML.LineString(
                KML.extrude("1"),
                KML.tessellate("1"),
                KML.altitudeMode("absolute"),
                KML.coordinates(self.execCoordListStr)
            )
        ))

        # check that the KML document is valid using the Google Extension XML Schema
        # assert(Schema("kml22gx.xsd").validate(self.tour_doc))

        # print etree.tostring(self.tour_doc, pretty_print=True)

        # output a KML file (named based on the Python script)
        #test = etree.tostring(self.tour_doc, pretty_print=True)
        #print(f'nasz string: {test}\n')
        outfile = open(kmlFilename, 'wb')
        outfile.write(etree.tostring(self.tour_doc, pretty_print=True))
        outfile.close()
        self.hasFinished = True

    # ---------------------------------------------------------------------------
    def update(self):
        print('dupa')
        if (self.hasOdom):
            # ----------------------------------
            dt = self.currOdom.header.stamp.to_sec() - self.prevOdom.header.stamp.to_sec()

            dx = self.currOdom.latitude - self.prevOdom.latitude
            dy = self.currOdom.longitude - self.prevOdom.longitude
            dz = self.currOdom.altitude - self.prevOdom.altitude

            d = 1000.0 * math.sqrt(dx * dx + dy * dy + dz * dz)
            print(f'delta^2:{d}')

            # v = d/dt
            # ----------------------------------
            if ((dt > 10.0) or
                    ((dt > 1.0) and (d > 10.0))):
                latLongPos = self.convertToLatLong(self.currOdom, self.currOrient)
                self.kml_addPathPoint(latLongPos, dt)
                self.totalTime = self.totalTime + dt
                self.prevOdom = self.currOdom
                self.prevOrient = self.currOrient
                rospy.loginfo("Total time = {0}".format(self.totalTime))

            self.hasOdom = False
            self.hasOrient = False

    # ---------------------------------------------------------------------------
    def convertToLatLong(self, gps, odom):
        utm_north = gps.latitude
        utm_east = gps.longitude

        rospy.loginfo("UTM: {0} N / {1} E".format(utm_north, utm_east))
        # print(f'North: {utm_north}, East:{utm_east}')
        # latlon = utm.to_latlon(utm_north, utm_east, self.utmZoneNumber, self.utmZoneLetter)
        # rospy.logerr(latlon)
        latlon = [utm_north, utm_east]
        latitude = latlon[0]
        longitude = latlon[1]
        altitude = -gps.altitude + self.zOffset

        ori = odom.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(([ori.x, ori.y, ori.z, ori.w]))
        # rospy.logerr(euler)
        return (latitude, longitude, altitude, euler[0] * (180.0 / math.pi), euler[1] * (180.0 / math.pi),
                euler[2] * (180.0 / math.pi))

    # ---------------------------------------------------------------------------
    def kml_addPathPoint(self, pos, dt):
        # pos[0] : latitude
        # pos[1] : longitude
        # pos[2] : altitude
        # pos[3] : roll
        # pos[4] : pitch
        # pos[5] : yaw
        if (pos == None):
            return
        # rospy.logerr(pos)

        self.tour_doc.Document[self.gxns + "Tour"].Playlist.append(
            GX.FlyTo(
                GX.duration(dt),
                GX.flyToMode("smooth"),
                # KML.Camera(
                KML.LookAt(
                    KML.latitude(pos[0]),
                    KML.longitude(pos[1]),
                    KML.altitude(pos[2]),
                    KML.heading(pos[5]),
                    # KML.tilt(pos[4] + 90.0),
                    # KML.roll(pos[3]),
                    KML.tilt(pos[4] + 75.0),
                    KML.range(20.0),
                    KML.altitudeMode("absolute"),
                ),
            ),
        )

        auxStr = ' {lon},{lat},{alt}\n'.format(lon=pos[1], lat=pos[0], alt=pos[2])
        # rospy.logerr(auxStr)
        self.execCoordListStr = self.execCoordListStr + auxStr

    def kml_addAnimPoint(self, pos, tm):
        # pos[0] : latitude
        # pos[1] : longitude
        # pos[2] : altitude
        # pos[3] : roll
        # pos[4] : pitch
        # pos[5] : yaw
        if (pos == None):
            return

        localTimeStr = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.localtime(tm))
        # rospy.logerr(localTimeStr)
        self.anim_fld.append(KML.Placemark(
            KML.TimeStamp(
                KML.when(localTimeStr)
            ),
            KML.altitudeMode("absolute"),
            KML.styleUrl('#sn_shaded_dot'),
            KML.LookAt(
                KML.longitude(pos[1]),
                KML.latitude(pos[0]),
                KML.altitude(pos[2]),
                KML.heading(pos[5]),
                KML.tilt(0.0),
                KML.range(100.0),
                KML.altitudeMode("absolute"),
            ),
            KML.Point(
                KML.coordinates("{lon},{lat},{alt}".format(lon=pos[1], lat=pos[0], alt=pos[2]))
            )
        ))

    # -------------------------------------------------------------------------------


# -------------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        rospy.init_node('kml_tour_generator')

        kmlTourGen = None

        # -----------------------------------------------------------------------
        kmlOutputFilename = rospy.get_param('/generate_kml/output_filename')

        # -----------------------------------------------------------------------
        # Topic name
        pose_topic = rospy.get_param('/generate_kml/pose_topic')
        orient_topic = rospy.get_param('/generate_kml/orient_topic')
        # UTM zone number used for coordinate transformation
        utmZoneNumber = rospy.get_param('/generate_kml/utm_zone_number')

        # -----------------------------------------------------------------------
        # Get pose information and build KML file
        kmlTourGen = KMLTourGenerator(pose_topic, orient_topic, utmZoneNumber)

        r = rospy.Rate(5.0)
        while (not rospy.is_shutdown()):
            kmlTourGen.update()
            r.sleep()
        print(f'filename: {kmlOutputFilename}\n')
        kmlTourGen.finish(kmlOutputFilename)

    except rospy.ROSInterruptException:
        if (kmlTourGen != None):
            kmlTourGen.finish(kmlOutputFilename)
        pass
