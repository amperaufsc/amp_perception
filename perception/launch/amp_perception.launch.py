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
    left = "OAKDLR_left"
    right = "OAKDLR_right"
    
    cam_info_left = PathJoinSubstitution([FindPackageShare('perception'), 'config',
                                                  left+'.yaml'])
    cam_info_right = PathJoinSubstitution([FindPackageShare('perception'), 'config',
                                                  right+'.yaml'])
    
    
    return LaunchDescription([
        LaunchArg('namespace',default_value=['namespace'],description='namespace for Node'),
        LaunchArg('disparity',default_value=['disparity'],description='disparity img topic'),
        LaunchArg('inference',default_value=['inference'],description='yolo inference topic'),
        LaunchArg('camera/left',default_value=['camera/left'],description='camera left topic'),
        LaunchArg('camera/right',default_value=['camera/right'],description='camera right topic'),
        LaunchArg('track',default_value=['track'],description='track msg topic'),
        LaunchArg('pointcloud',default_value=['pointcloud'],description='pointcloud msg topic'),
        LaunchArg('left_camera_info',default_value=['file://', cam_info_left],
                  description='camera left info with intrinsics and distortion matrix'
        ),
        LaunchArg('left_camera_info', default_value=['file://', cam_info_right], 
                  description='camera right info with intrinsics and distortion matrix'
                  ),
        Node(
            package='perception',
            executable='depthai_position_estimator.py',
            name='depthai_position_estimator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('disparity', LaunchConfiguration('disparity')),
                ('inference', LaunchConfiguration('inference')),
                ('camera/left', LaunchConfiguration('camera/left')),
                ('camera/right', LaunchConfiguration('camera/right')), 
                ('track', LaunchConfiguration('track')),
                ('pointcloud',LaunchConfiguration('pointcloud'))
            ]
        )
        ])
