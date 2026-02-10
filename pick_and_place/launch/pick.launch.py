from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
from math import radians
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ArmPlate",package_name="arm_urdf_moveit_config").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="pick_and_place",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )
    return LaunchDescription([pick_place_demo])