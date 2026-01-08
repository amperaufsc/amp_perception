from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='point', description='Namespace for node'),
        #DeclareLaunchArgument('camera_sub', default_value='/fsds/cameracam2/camera_info', description='Camera Info message topic'),
        DeclareLaunchArgument('image_sub', default_value='/fsds/cameracam2/image_color', description='Image Info message topic'),
        DeclareLaunchArgument('lidar_sub', default_value='/fsds/lidar/Lidar1', description='LiDAR Info message topic'),
        DeclareLaunchArgument('lidar_pub', default_value='/fusion/lidar_camera', description="LiDAR/Camera fusion message topic"),

        Node(
            package='lidar_filtering',
            executable='lidar_fusion',
            name='lidar_fusion',
            namespace=LaunchConfiguration('namespace'),
            remappings=[
                #('/fsds/cameracam2/camera_info', LaunchConfiguration('camera_sub')),
                ('image_sub', LaunchConfiguration('image_sub')),
                ('lidar_sub', LaunchConfiguration('lidar_sub')),
                ('lidar_pub', LaunchConfiguration('lidar_pub'))
            ],
            #parameters=[{'frame_id': LaunchConfiguration('frame_id')}]
        )
    ])
