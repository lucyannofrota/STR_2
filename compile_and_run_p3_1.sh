
#!/bin/sh

cd ros_foxy
colcon build

ros2 launch str_ROS2 launch_strNode_1.py