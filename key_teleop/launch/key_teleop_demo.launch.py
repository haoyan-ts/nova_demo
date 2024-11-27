from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dobot_bringup_v3',
            executable='dobot_bringup',
            name='dobot_bringup_v3',
            # output='screen',
            # parameters=[
            #     {'param_name': 'param_value'}
            # ],
            # remappings=[
            #     ('/old_topic', '/new_topic')
            # ]
        ),
        Node(
            package='key_teleop',
            executable='key_teleop',
            name='key_teleop',
            # output='screen',
            # parameters=[
            #     {'param_name': 'param_value'}
            # ],
            # remappings=[
            #     ('/old_topic', '/new_topic')
            # ]
        ),
    ])