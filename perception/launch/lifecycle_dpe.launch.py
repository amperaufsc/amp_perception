from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, EmitEvent
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    lifecycle_node = LifecycleNode(
        package='perception',
        executable='lifecycle_dpe.py',
        name='perception_node',
        namespace='',
        output='screen',
        parameters=[{
            # Exemplo de parâmetros corretos
            'left_camera_info': 'src/as_amp/perception/config/OAKDLR_left.yaml',
            'right_camera_info': 'src/as_amp/perception/config/OAKDLR_right.yaml',
            'set_disparity': True
        }]
    )

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

    return LaunchDescription([
        lifecycle_node,
        TimerAction(period=1.0, actions=[configure_event, LogInfo(msg='Configurando Perception')]),
        TimerAction(period=5.0, actions=[activate_event, LogInfo(msg='Ativando Perception')])
    ])
