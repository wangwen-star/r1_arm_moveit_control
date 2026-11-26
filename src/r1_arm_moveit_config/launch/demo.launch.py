#from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_demo_launch


#def generate_launch_description():
#    moveit_config = MoveItConfigsBuilder("R1_Robotic_Arm", package_name="r1_arm_moveit_config").to_moveit_configs()
#    return generate_demo_launch(moveit_config)
    
    
    
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("R1_Robotic_Arm", package_name="r1_arm_moveit_config")
        # 关键！关闭自动管理ros2_control控制器，不启动controller_manager
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)

