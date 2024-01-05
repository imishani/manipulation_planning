# A launch file for ros2 run manipulation_planning manipulation_planning_cspace_test

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('manipulation_planning'), 'launch')
    rviz_dir = os.path.join(get_package_share_directory('manipulation_planning'), 'rviz')
    # Create the launch configuration variables
    ld = LaunchDescription([
            Node(
                package='manipulation_planning', 
                executable='manipulation_planning_mramp_test', 
                output='screen',
            )
        ]
    )
        
    return ld
