# ROS2 Interface plugin for CoppeliaSim

### Compiling

_NOTE:_ the directory containing all files (i.e. package.xml etc) must be called sim_ros2_interface, otherwise build will fail.

1. Install required packages for [libPlugin](https://github.com/CoppeliaRobotics/libPlugin): see libPlugin's [README](external/libPlugin/README.md)
2. Checkout
```
$ git clone --recursive https://github.com/CoppeliaRobotics/simExtRos2Interface.git sim_ros2_interface
```
3. Edit `meta/messages.txt` and `meta/services.txt` if you need to include more ROS messages/services. You need to specify the full message/service type, i.e. geometry_msgs/Twist rather than Twist.
4. Compile
```
$ catkin build
```
