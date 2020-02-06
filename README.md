**NOTE:** This repository is related with the next scientific work:

**Barral, V.**; **Escudero, C.J.**; **García-Naya, J.A.**; **Maneiro-Catoira, R.** *NLOS Identification and Mitigation Using Low-Cost UWB Devices.* Sensors 2019, 19, 3464.[https://doi.org/10.3390/s19163464](https://doi.org/10.3390/s19163464)

If you use this code for your scientific activities, a citation is appreciated.

# README

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


