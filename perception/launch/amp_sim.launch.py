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
    left = "/config/fsds_left"
    right = "/config/fsds_right"

    disparity_set = False
    
    cam_info_left = "/home/pedro_garbin/ws/src/as_amp/perception/config/fsds_left.yaml"
    cam_info_right = "/home/pedro_garbin/ws/src/as_amp/perception/config/fsds_right.yaml"
    frame_id  = "/camera_link"
    
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
            executable='position_estimator.py',
            name='position_estimator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('disparity', LaunchConfiguration('disparity')),
                ('inference', LaunchConfiguration('inference')),
                ('camera/left', LaunchConfiguration('camera/left')),
                ('camera/right', LaunchConfiguration('camera/left')), 
                ('track', LaunchConfiguration('track')),
                ('pointcloud',LaunchConfiguration('pointcloud'))
            ],
            parameters=[
            {'left_camera_info': cam_info_left},
            {'right_camera_info': cam_info_right},
            {'set_disparity': disparity_set},
            {'frame_id': frame_id}
        ]
        ),
        

        ExecuteProcess(
    cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
         '0', '0', '0',  # x, y, z 
         '-1.570796327',  # yaw
         '0',  # pitch
         '-1.570796327',  # roll
         '/fsds/cam2',  # frame_id
         frame_id],  # child_frame_id
    output='screen')
        
    ])
