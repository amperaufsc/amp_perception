from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os
   
def generate_launch_description():

    return LaunchDescription([
            LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
            LaunchArg('inferenceresult',default_value=['inferenceresult'],description='yolo inference topic'),
            LaunchArg('image/left',default_value=['image/left'],description='camera left topic'),
            LaunchArg('image/right',default_value=['image/right'],description='camera right topic'),
            LaunchArg('track_pub',default_value=['track_pub'],description='track msg topic'),
            Node(
                package='perception',
                executable='cone_track_node.py',
                name='Cone_Track_Process',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                remappings=[
                    ('inferenceresult', LaunchConfiguration('inferenceresult')),
                    ('image/left', LaunchConfiguration('image/left')),
                    ('image/right', LaunchConfiguration('image/right')), 
                    ('track_pub', LaunchConfiguration('track_pub')),
                ]
            )
            ])