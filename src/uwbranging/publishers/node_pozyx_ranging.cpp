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
#include "PozyxRangingReader.h"


int main(int argc, char *argv[])
{


    ros::init(argc, argv, "pozyxranging");
    ros::NodeHandle n("~");
    ros::Publisher pozyx_ranging_pub = n.advertise<gtec_msgs::PozyxRanging>("/gtec/uwb/ranging/pozyx", 1000);

    std::string usbPort;
    n.getParam("usbPort", usbPort);

    std::string angleTopic;
    n.getParam("angleTopic", angleTopic);

    int useAngle;
    n.getParam("useAngle", useAngle);


    PozyxRangingReader pozyxRangingReader;

    ros::Subscriber sub;
    if (useAngle==1){
        sub = n.subscribe<std_msgs::Float64>(angleTopic, 20, &PozyxRangingReader::newAngle, &pozyxRangingReader);
    }

    pozyxRangingReader.start(usbPort, pozyx_ranging_pub);

    ros::spin();


    return 0;
}
