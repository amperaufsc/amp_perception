from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='perception',
            executable='depthai_camera_node.py',
            name='depthai_position_estimator',
            output='screen',
            remappings=[
            ]
        ),
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                    '--yaw', '-1.570796327',
                    '--roll', '-1.5707963270',
                    '--pitch', '0',
                    '--frame-id', 'down',
                    '--child-frame-id', 'camera_link'],
            output='screen'
        )
        
    ])