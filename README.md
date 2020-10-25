**NOTE:** This repository is related with the next scientific work:

**Barral, V.**; **Escudero, C.J.**; **García-Naya, J.A.**; **Maneiro-Catoira, R.** *NLOS Identification and Mitigation Using Low-Cost UWB Devices.* Sensors 2019, 19, 3464.[https://doi.org/10.3390/s19163464](https://doi.org/10.3390/s19163464)

If you use this code for your scientific activities, a citation is appreciated.

# README

This package include some nodes to read range and position values coming from Pozyx devices, and to publish them in ROS topics.

**Important:** All the nodes in this package work with a tag attached to the host, but this device acts only as relay of the measurements coming from another remote tag. The typical setup is: 
- You have a tag connected to your ROS machine (some nodes require an Arduino shield connected to the tag).
- You have your anchors placed in your room or area of interest.
- You have another tag moving around in the room. 
- The data that you can get in your ROS machine will be the corresponding to the moving tag, **not** the corresponding to the tag connected to your ROS machine.


## PozyxRangingReader

This node reads the ranging values outputted by a Pozyx device connected to and Arduino board that runs the corresponding script available in [https://github.com/valentinbarral/pozyxarduinoscripts.git](https://github.com/valentinbarral/pozyxarduinoscripts.git). The ranging values are published in ROS under the topic */gtec/uwb/ranging/pozyx* and the message type is ```gtec_msgs::PozyxRanging``` (this repository is required: [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs)).

To launch the node can be used the launcher *pozyxranging.launch*. It has one argument *usbPort* to indicate the USB port where the Pozyx (through an Arduino shield) is connected. A typical example of use could be:

```
$ roslaunch gtec_uwb_ranging pozyxranging.launch usbPort:=/dev/ttyUSB0
```
## PozyxRangingIMU

This node is similar to the previous one, but in this case the output also contains the IMU data from the Pozyx. It requires the corresponding Arduino script loaded in the shield, available here: [https://github.com/valentinbarral/pozyxarduinoscripts.git](https://github.com/valentinbarral/pozyxarduinoscripts.git) The new topic with the IMU measurements is */gtec/pozyx/imu* (the ranging measurements remain published in */gtec/uwb/ranging/pozyx*).

## PozyxRangingCIR

This node also works with a Pozyx connected to an Arduino shield, but in this case the outputs contains the CIR (Channel Impulse Response) measurements after each ranging operation. The measurements are published in ROS under the topic */gtec/uwb/rangingwithcir/pozyx* and the message type is ```gtec_msgs::PozyxRangingWithCir```. This message type can be found here: [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs). Again, is required to load the corresponding script in the Arduino, available here: [https://github.com/valentinbarral/pozyxarduinoscripts.git](https://github.com/valentinbarral/pozyxarduinoscripts.git)

## PozyxPosition

This node inserts in a ROS environment the positions calculated by the internal algorithms of the Pozyx device. The messages are of type ```geometry_msgs::PoseWithCovarianceStamped``` Again, this version works with an Arduino Shield. The positions are published under the topic */gtec/uwb/position/pozyx/TAG_ID*, where *TAG_ID* is the internal identifier of the tracked tag and can be set in the parameters of the node.

## PozyxPositionReaderPython

This is a Python node to read the positions from a connected Pozyx device. In this case is **not** necessary an Arduino shield, so the tag can be connected straight to the host machine. The node can be configured with the tagId, the algorithm or the dimension to use. The positions are published in */gtec/uwb/position/pozyx/TAG_ID*. In this case, this node uses the node *anchors* from the package *gtec_rostoa* available here: [https://github.com/valentinbarral/rostoa](https://github.com/valentinbarral/rostoa). This node can be configured with the positions of the anchors and it publishes these position in a ROS topic. This way, you can use at the same time different location algorithms like this *PozyxPositionReaderPython* but also others like the available here: [https://github.com/valentinbarral/roskfpos](https://github.com/valentinbarral/roskfpos)

### Installation

As usual, inside a catkin workspace and of course after cloning the repository:

```
$ catkin_make
```


