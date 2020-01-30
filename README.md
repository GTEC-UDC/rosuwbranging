# ROS UWB RANGING

This package include some nodes to read range values coming from different UWB devices.

## PozyxRangingReader

This node reads the ranging values outputted by a Pozyx device connected to and Arduino board that runs the corresponding script available in [https://github.com/valentinbarral/pozyxarduinoscripts.git](https://github.com/valentinbarral/pozyxarduinoscripts.git). The ranging values are published under the topic */gtec/uwb/ranging/pozyx* and the message type is ```gtec_msgs::PozyxRanging``` (this repository is required: [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs)).

To launch the node can be used the launcher *pozyxranging.launch*. It has one argument *usbPort* to indicate the USB port where the Pozyx (through an Arduino shield) is connected. A typical example of use could be:

```
$ roslaunch gtec_uwb_ranging pozyxranging.launch usbPort:=/dev/ttyUSB0
```

### Installation

As usual, inside a catkin workspace and of course after cloning the repository:

```
$ catkin_make
```


