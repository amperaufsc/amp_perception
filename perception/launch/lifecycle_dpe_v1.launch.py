from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument as LaunchArg
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from launch.actions import TimerAction, LogInfo, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessExit
   
class Respawn:
    
    def callback(self, event, context):
        return [
            LogInfo(msg='Enviando shutdown pro MAPPER.'),
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == lifecycle_node,
                    transition_id=Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
                )
            )
                
        ]
       
def generate_launch_description():
    global lifecycle_node
    lifecycle_node = LifecycleNode(
        package='perception',
        executable='lifecycle_dpe_v1.py',
        name='position_estimator',
        namespace='',
        output='screen',
        parameters=[{
            # Exemplo de parâmetros corretos
            'left_camera_info': 'src/as_amp/perception/config/OAKDLR_left.yaml',
            'right_camera_info': 'src/as_amp/perception/config/OAKDLR_right.yaml',
            'set_disparity': True
        }]
    )
    counter = Respawn()

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == lifecycle_node,
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == lifecycle_node,
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )

    on_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_node,
            on_exit=counter.callback
        )
    )

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
        lifecycle_node,
        TimerAction(period=3.0, actions=[configure_event, LogInfo(msg='Configurando Perception')]),
        TimerAction(period=5.0, actions=[activate_event, LogInfo(msg='Ativando Perception')]),
        on_exit_handler
        ])
