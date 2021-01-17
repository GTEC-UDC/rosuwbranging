#!/usr/bin/env python


""" MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. """


import rospy, time, serial, os, tf2_ros
from time import sleep

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, 
                     PozyxRegisters, DeviceRange, EulerAngles, Acceleration, Quaternion,AngularVelocity)
                     
from pypozyx.tools.version_check import perform_latest_version_check

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from gtec_msgs.msg import Ranging
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

class PozyxRanger(object):
    def __init__(self, pozyxSerial, ros_rate, publisher_ranging, targetDeviceId):
        self.targetDeviceId = targetDeviceId
        self.rate = ros_rate
        self.r = ReadyToRange(pozyxSerial, publisher_ranging, targetDeviceId)
        self.hasAnchors = False

    def setAnchors(self,anchors):
        self.r.setAnchors(anchors)
        while not rospy.is_shutdown():
            self.r.loop()
            self.rate.sleep()

    def callbackAnchorsMessage(self, anchorsMsg):
        if not self.hasAnchors:
            print("Anchors Msg received")
            anchors = []
            for anchor in anchorsMsg.markers:
                deviceCoordinates = DeviceCoordinates(anchor.id,1, Coordinates(anchor.pose.position.x*1000, anchor.pose.position.y*1000, anchor.pose.position.z*1000))
                anchors.append(deviceCoordinates)
            self.setAnchors(anchors)
            self.hasAnchors = True


class ReadyToRange(object):

    def __init__(self, pozyx, ranging_pub, remote_id=None):
        self.pozyx = pozyx
        self.remote_id = remote_id
        self.ranging_pub = ranging_pub
        self.seq = -1


    def setAnchors(self, anchors):
        self.anchors = anchors
        self.printPublishAnchorConfiguration()

    def loop(self):

        self.seq += 1
        if self.seq>255:
            self.seq=0
        current_seq = self.seq
        for anchor in self.anchors:
            range = DeviceRange()
            try:
                status = self.pozyx.doRanging(anchor.network_id, range, self.remote_id)
                if status == POZYX_SUCCESS and range.RSS<0.0:
                    self.publishRanging(anchor.network_id, range, current_seq)
            except:
                print("################## An exception occurred #########################")
                #self.pozyx.resetSystem()
                serial_port = get_first_pozyx_serial_port()
                if serial_port is None:
                    print("No Pozyx connected. Check your USB cable or your driver!")
                    quit()
                self.pozyx = PozyxSerial(serial_port)

    def publishRanging(self, anchorId, range, seq):
        ranging = Ranging()
        ranging.anchorId = anchorId
        ranging.tagId = self.remote_id
        ranging.range = range.distance
        ranging.rss = range.RSS
        ranging.seq = seq
        ranging.errorEstimation = 0.00393973
        self.ranging_pub.publish(ranging)


    def printPublishAnchorConfiguration(self):

        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            sleep(0.025)


    

if __name__ == "__main__":

    rospy.init_node('PozyxRangingReaderPython', anonymous=True)

    # Read parameters
    targetDeviceIdString = rospy.get_param('~targetDeviceId')
    serial_port = rospy.get_param('~serial')


    targetDeviceId = int(targetDeviceIdString,16)
    

    pub_ranging = rospy.Publisher("/gtec/uwb/ranging/pozyx", Ranging, queue_size=100)
    rate = rospy.Rate(5) # 10hz


    print("=========== POZYX Range reader ============")
    print("targetDeviceId: " + targetDeviceIdString)
    print("=========== [---------------] ============")

    check_pypozyx_version = False
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself
    #serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    pozyxSerial = PozyxSerial(serial_port)
    pozyxRanger = PozyxRanger(pozyxSerial, rate, pub_ranging, targetDeviceId)


    rospy.Subscriber('/gtec/toa/anchors', MarkerArray, pozyxRanger.callbackAnchorsMessage)

    print("Waiting anchors positions...")
    rospy.spin()


