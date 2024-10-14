from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

package_name = 'csmbus'

def generate_launch_description():
    subprocess.run(["blackbox_create"]) 

    ld = LaunchDescription()
    
    ld.add_action(Node(
        package=package_name,
        namespace='',
        executable='csmbus',
        parameters=[
        ]
    ))
    return ld