import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('manipulation_planning')

    # Declare the launch arguments
    declare_planner_name_cmd = DeclareLaunchArgument(
        'planner_name',
        default_value='xECBS',
        description='Planner name'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # Print the value of the argument planner_name
    print_planner_name_cmd = LogInfo(msg=LaunchConfiguration('planner_name'))

    # Create the manipulation planning node
    mramp_node = Node(
        package='manipulation_planning',
        executable='manipulation_planning_mramp_test',
        output='screen',
        parameters=[{
            'planner_name': LaunchConfiguration('planner_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        declare_planner_name_cmd,
        declare_use_sim_time_cmd,
        print_planner_name_cmd,
        mramp_node
    ])

    return ld
