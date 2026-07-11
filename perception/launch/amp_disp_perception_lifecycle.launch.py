from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument as LaunchArg
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
   
def generate_launch_description():
    arquivo_left = "OAKDLR_left_22_04.yaml"
    arquivo_right = "OAKDLR_right_22_04.yaml"

    return LaunchDescription([
        LaunchArg('namespace',default_value=[''],description='namespace for Node'),
        LaunchArg('disparity',default_value=['disparity'],description='disparity img topic'),
        LaunchArg('inference',default_value=['inference'],description='yolo inference topic'),
        LaunchArg('camera/left',default_value=['/oak/left/image_raw'],description='camera left topic'),
        LaunchArg('camera/right',default_value=['/oak/right/image_raw'],description='camera right topic'),
        LaunchArg('track',default_value=['track'],description='track msg topic'),
        LaunchArg('left_config_file_name',default_value=[arquivo_left],description='Intrinsic/extrinsic left camera matrix yaml file name'),
        LaunchArg('right_config_file_name',default_value=[arquivo_right],description='Intrinsic/extrinsic right camera matrix yaml file name'),
 
        Node(
            package='perception',
            executable='track_node_pub_life.py',
            name='perception_lifecycle_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('disparity', LaunchConfiguration('disparity')),
                ('inference', LaunchConfiguration('inference')),
                ('camera/left', LaunchConfiguration('camera/left')),
                ('camera/right', LaunchConfiguration('camera/right')), 
                ('track', LaunchConfiguration('track'))
            ]
        )
        ])

