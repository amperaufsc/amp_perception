from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration

# TODO: Study how to change an argument in ROS2
MODEL_NAME = "best.pt"

def generate_launch_description():

    # Confidene set according to Precision-Recall graph gotten from yolo training
    # TODO: establish with more accuracy the good confidence
    confidence = 0.5
    yolo_path = f'src/amp_perception/yolobot_recognition/scripts/{MODEL_NAME}'

    return LaunchDescription([
        LaunchArg('namespace', default_value=[''], description='Namespace for node'),
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
            {'yolo_path': yolo_path}
        ]
    )
    ])