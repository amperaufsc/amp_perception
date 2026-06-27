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
    
    cam_info_left = os.path.join(
        get_package_share_directory('perception'),
        'config', 
        arquivo_left
    )

    cam_info_right = os.path.join(
        get_package_share_directory('perception'),
        'config', 
        arquivo_right
    )

    return LaunchDescription([
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('disparity',default_value=['disparity'],description='disparity img topic'),
        LaunchArg('inference',default_value=['inference'],description='yolo inference topic'),
        LaunchArg('camera/left',default_value=['camera/left'],description='camera left topic'),
        LaunchArg('camera/right',default_value=['camera/right'],description='camera right topic'),
        LaunchArg('track',default_value=['track'],description='track msg topic'),

        Node(
            package='perception',
            executable='perception_lifecycle_node.py',
            name='perception_lifecycle_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('disparity', LaunchConfiguration('disparity')),
                ('inference', LaunchConfiguration('inference')),
                ('camera/left', LaunchConfiguration('camera/left')),
                ('camera/right', LaunchConfiguration('camera/right')), 
                ('track', LaunchConfiguration('track'))
            ],
            parameters=[
                {'left_camera_info': cam_info_left},
                {'right_camera_info': cam_info_right}
            ]
        )
        ])

