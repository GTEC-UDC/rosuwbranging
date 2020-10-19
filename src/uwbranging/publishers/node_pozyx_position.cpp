/*
MIT License

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
SOFTWARE.
*/
#include "PozyxPositionReader.h"


int main(int argc, char *argv[])
{


    ros::init(argc, argv, "pozyxposition");
    ros::NodeHandle n("~");

    std::string usbPort,targetDeviceId;
    n.getParam("usbPort", usbPort);
    n.getParam("targetDeviceId", targetDeviceId);

    std::ostringstream stringStream;
    stringStream << "/gtec/uwb/position/pozyx/" << targetDeviceId.c_str();
    std::string topicPos = stringStream.str();

    ros::Publisher pozyx_position_pub =n.advertise<geometry_msgs::PoseWithCovarianceStamped>(topicPos, 1000);

    PozyxPositionReader pozyxPositionReader;
    pozyxPositionReader.start(usbPort, pozyx_position_pub);

    ros::spin();

    return 0;
}