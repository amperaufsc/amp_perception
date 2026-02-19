from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    confidence = 0.7
    yolov8_path = 'src/amp_perception/yolobot_recognition/scripts/best.pt'

    return LaunchDescription([
        LaunchArg('namespace', default_value=['yolov8'], description='Namespace for node'),
        LaunchArg('image',default_value=['image'],description='img topic'),
        LaunchArg('inferenceresult',default_value=['inferenceresult'],description='bounding box coordenates on image'),
        LaunchArg('inferenceimg',default_value=['inferenceimg'],description='img with boundingbox'),
        Node(
            package='yolobot_recognition',
            executable='yolov8_ros2_pt.py',
            name='yolobot_recognition',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('image',LaunchConfiguration('image')),
                ('inferenceresult',LaunchConfiguration('inferenceresult')),
                ('inferenceimg',LaunchConfiguration('inferenceimg'))
                ],
            parameters=[
            {'confidence_threshold': confidence},
            {'yolov8_path': yolov8_path}
        ]
    )
    ])