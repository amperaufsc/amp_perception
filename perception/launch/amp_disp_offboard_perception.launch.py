from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument as LaunchArg
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
   
def generate_launch_description():
    
    return LaunchDescription([
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('disparity',default_value=['disparity'],description='disparity img topic'),
        LaunchArg('image_rect/left',default_value=['image_rect/left'],description='camera left topic'),
        LaunchArg('image_rect/right',default_value=['image_rect/right'],description='camera right topic'),
        LaunchArg('track',default_value=['track'],description='camera right topic'),
        LaunchArg('inferenceresult',default_value=['inferenceresult'],description='camera right topic'),

        Node(
            package='perception',
            executable='cone_track_node.py',
            name='track_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('image_rect/left', LaunchConfiguration('image_rect/left')),
                ('image_rect/right', LaunchConfiguration('image_rect/right')),
                ('inferenceresult', LaunchConfiguration('inferenceresult')),
                ('disparity', LaunchConfiguration('disparity')),    
                ('track', LaunchConfiguration('track'))
            ]
        )
        ])
