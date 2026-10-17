"""go2_mpc viewer: UDP debug stream -> ROS 2, robot model, RViz2.

    ros2 launch go2_mpc_viz viz.launch.py            # with RViz2
    ros2 launch go2_mpc_viz viz.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = get_package_share_directory("go2_description")
    viz_dir = get_package_share_directory("go2_mpc_viz")
    with open(os.path.join(description_dir, "urdf", "go2.urdf")) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("port", default_value="9870"),
        Node(
            package="go2_mpc_viz",
            executable="go2_mpc_viz",
            parameters=[{"port": LaunchConfiguration("port")}],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(viz_dir, "rviz", "go2_mpc.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
