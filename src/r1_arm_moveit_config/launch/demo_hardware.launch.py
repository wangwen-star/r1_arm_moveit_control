import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("R1_Robotic_Arm", package_name="r1_arm_moveit_config")
        # 真臂:用带硬件插件的 URDF(相对 config 包路径,MoveItConfigsBuilder 会拼到 share 目录)
        # 注意:不要 moveit_manage_controllers=False——保持默认,等 JTC 激活后才执行轨迹,多一层保护
        .robot_description(file_path="config/R1_Robotic_Arm_hardware.urdf.xacro")
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("r1_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # ros2_control 节点(加载硬件插件 + 控制器管理器)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path, moveit_config.robot_description],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    # robot_state_publisher:/joint_states → TF(RViz 白色臂镜像真机的关键一环)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(
            get_package_share_directory("r1_arm_moveit_config"),
            "config",
            "moveit.rviz"
        )],
        # 必须传完整 MoveIt 配置(含 robot_description_semantic/SRDF),
        # 只传 robot_description 时 MotionPlanning 面板无法加载机器人模型
        parameters=[moveit_config.to_dict()],
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        ros2_control_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        spawn_joint_state_broadcaster,
        spawn_trajectory_controller
    ])
