import launch
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="str_ROS2", executable="str_node", output="screen"
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d'+'src/str_ROS2/config/str_rviz2.rviz']
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2','bag','play','2017-10-31-22-06-52/']
        )
    ])