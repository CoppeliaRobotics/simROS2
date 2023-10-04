# ROS2 Interface plugin for CoppeliaSim

Supported ROS2 versions:

 - Humble Hawksbill

### Compiling

_NOTE:_ the directory containing all files (i.e. package.xml etc) must be called sim_ros2_interface, otherwise build will fail.

1. Install required packages for simStubsGen: see simStubsGen's [README](https://github.com/CoppeliaRobotics/include/blob/master/simStubsGen/README.md)
2. Checkout
```
$ git clone https://github.com/CoppeliaRobotics/simROS2.git sim_ros2_interface
$ cd sim_ros2_interface
$ git checkout coppeliasim-v4.5.0-rev0
```

NOTE: replace `coppeliasim-v4.5.0-rev0` with the actual CoppeliaSim version you have.

3. Edit `meta/interfaces.txt` if you need to include more ROS interfaces. You need to specify the fully qualified interface, e.g. geometry_msgs/msg/Twist rather than Twist. If an interface uses non-primitive types (i.e. other interfaces), then those should be added as well.
4. Compile
```
$ colcon build --symlink-install
```

Note: if you are reporting a compile error, please use this command to build:
```
VERBOSE=1 MAKEFLAGS=-j1 colcon build --symlink-install --event-handlers console_direct+ --parallel-workers 1
```

Add `--cmake-args -DCMAKE_BUILD_TYPE=Debug` if you are encountering a runtime error (e.g. crash, unexpected behavior, etc...).

Note: *gcc* can fail compile the plugin when a large number of interfaces is compiled in. Use *clang* in that case, i.e.:

 ```
 sudo apt install clang
 export CXX=clang++
 colcon build ...
 ```
