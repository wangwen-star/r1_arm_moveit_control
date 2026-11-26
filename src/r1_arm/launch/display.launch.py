from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("r1_arm")
    urdf_file = os.path.join(pkg_share, "urdf", "R1_Robotic_Arm.urdf")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf_file).read()}]
    )

    joint_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_gui_node,
        rviz2_node
    ])
