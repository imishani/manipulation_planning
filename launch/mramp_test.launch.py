# A launch file for ros2 run manipulation_planning manipulation_planning_cspace_test
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('manipulation_planning'), 'launch')
    rviz_dir = os.path.join(get_package_share_directory('manipulation_planning'), 'rviz')

    # Declare the launch arguments.
    declare_planner_name_cmd = DeclareLaunchArgument(
        'planner_name',
        default_value='xECBS',
        description='Planner name'
    )


    # Print the value of the argument planner_name.
    print_planner_name_cmd = LogInfo(msg=LaunchConfiguration('planner_name'))

    # Create the manipulation planning node.
    mramp_node = Node(
        package='manipulation_planning', 
        executable='manipulation_planning_mramp_test', 
        output='screen',
        parameters=[{'planner_name':LaunchConfiguration('planner_name')}]
       )

    # Create the launch configuration variables
    ld = LaunchDescription( [declare_planner_name_cmd, print_planner_name_cmd, mramp_node] )
        
    return ld
