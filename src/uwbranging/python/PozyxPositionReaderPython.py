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


import rospy, time, serial, os
from time import sleep

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, PozyxRegisters)
                     
from pypozyx.tools.version_check import perform_latest_version_check

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray


class PozyxLocalizator(object):
    def __init__(self, pozyxSerial, ros_rate, publisher, targetDeviceId, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000):
        self.targetDeviceId = targetDeviceId
        self.rate = ros_rate
        self.r = ReadyToLocalize(pozyxSerial, publisher, algorithm, dimension, height, targetDeviceId)
        self.hasAnchors = False

    def setAnchors(self,anchors):
        self.r.setAnchors(anchors)
        self.r.setup()
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


class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""

    def __init__(self, pozyx, pose_pub, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        # self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
        self.pose_pub = pose_pub


    def setAnchors(self, anchors):
        self.anchors = anchors

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")
        print("- System will manually configure tag")
        print("")
        print("- System will auto start positioning")
        print("")
        if self.remote_id is None:
            self.pozyx.printDeviceInfo(self.remote_id)
        else:
            for device_id in [None, self.remote_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual(save_to_flash=False)
        self.printPublishConfigurationResult()

    def loop(self):
        """Performs positioning and displays/exports the results."""
        position = Coordinates()
        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            self.printPublishPosition(position)
        # else:
        #     self.printPublishErrorCode("positioning")

    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0

        p = PoseWithCovarianceStamped()
        p.pose.pose.position.x = position.x/1000.0
        p.pose.pose.position.y = position.y/1000.0
        p.pose.pose.position.z = position.z/1000.0
        # Make sure the quaternion is valid and normalized
        p.pose.pose.orientation.x = 0.0
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 1.0
        
        for i in range(36):
            p.pose.covariance[i] = 0.0
        
        p.header.frame_id = "world"
        p.header.stamp = rospy.Time.now()

        self.pose_pub.publish(p)

        #print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
        #    "0x%0.4x" % network_id, pos=position))

    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            # should only happen when not being able to communicate with a remote Pozyx.

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                       remote_id=self.remote_id)

        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            sleep(0.025)

    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            sleep(0.025)


    

if __name__ == "__main__":

    rospy.init_node('PozyxPositionReaderPython', anonymous=True)

    # Read parameters
    targetDeviceIdString = rospy.get_param('~targetDeviceId')
    algorithmString = rospy.get_param('~algorithm')
    dimensionString = rospy.get_param('~dimension')
    height = rospy.get_param('~height')

    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
    if algorithmString.startswith('POSITIONING_ALGORITHM_TRACKING'):
        algorithm = PozyxConstants.POSITIONING_ALGORITHM_TRACKING

    dimension = PozyxConstants.DIMENSION_3D
    if dimensionString.startswith('DIMENSION_2D'):
        dimension = PozyxConstants.DIMENSION_2D
    elif dimensionString.startswith('DIMENSION_2_5D'):
        dimension = PozyxConstants.DIMENSION_2_5D

    targetDeviceId = int(targetDeviceIdString,16)
    

    pub = rospy.Publisher('/gtec/uwb/position/pozyx/'+targetDeviceIdString, PoseWithCovarianceStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz


    print("=========== POZYX Localizator ============")
    print("targetDeviceId: " + targetDeviceIdString)
    print("algorithm: " + algorithmString)
    print("dimension: " + dimensionString)
    print("height: " + str(height))
    print("=========== [---------------] ============")

    # check_pypozyx_version = True
    # if check_pypozyx_version:
    #     perform_latest_version_check()

    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()
    pozyxSerial = PozyxSerial(serial_port)

    pozyxLocalizator = PozyxLocalizator(pozyxSerial,rate, pub, targetDeviceId, algorithm, dimension, height)


    rospy.Subscriber('/gtec/toa/anchors', MarkerArray, pozyxLocalizator.callbackAnchorsMessage)

    print("Waiting anchors positions...")
    rospy.spin()


