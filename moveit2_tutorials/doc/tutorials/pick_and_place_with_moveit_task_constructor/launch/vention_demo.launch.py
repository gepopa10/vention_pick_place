from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    pick_place_demo = Node(
        package="moveit2_tutorials",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )
    
    minotor = Node(
        package="monitor",
        executable="monitor",
        output="screen",
    )
    
    # Set the path to the launch files to include
    moveit2_tutorials_share_dir = get_package_share_directory('moveit2_tutorials')
    move_group_interface_launch_file = moveit2_tutorials_share_dir + '/launch/move_group_interface_tutorial.launch.py'
    move_group_launch_file = moveit2_tutorials_share_dir + '/launch/move_group.launch.py'

    # Define actions to launch the included launch files
    move_group_interface_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_interface_launch_file)
    )
    
    move_group_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_file)
    )

    return LaunchDescription([pick_place_demo, move_group_interface_action, move_group_action, minotor])
