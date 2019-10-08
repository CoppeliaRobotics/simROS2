# ROS Interface plugin for V-REP

### Compiling

_NOTE:_ the directory containing all files (i.e. package.xml etc) must be called vrep_ros_interface, otherwise build will fail.

1. Install required packages for [v_repStubsGen](https://github.com/CoppeliaRobotics/v_repStubsGen): see v_repStubsGen's [README](external/v_repStubsGen/README.md)
2. Checkout
```
$ git clone --recursive https://github.com/CoppeliaRobotics/v_repExtRosInterface.git vrep_ros_interface
```
3. Edit `meta/messages.txt` and `meta/services.txt` if you need to include more ROS messages/services. You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.
4. Compile
```
$ catkin build
```
