
#!/bin/sh

cd ros_foxy
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
. install/local_setup.bash

ros2 launch str_ROS2 launch_strNode_2.py