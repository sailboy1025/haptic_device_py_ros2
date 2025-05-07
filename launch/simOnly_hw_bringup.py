from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    inv3_node = Node(
        package='hd_py',
        executable='inv3_web',
        name='inv3_websocket_node',
        namespace='inv3',
        output='screen'
    )

    send_use_for_sim_msg = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 2; ros2 topic pub /inv3/use_for_coppeliasim std_msgs/msg/Bool "{data: true}" --once'
        ],
        shell=True,
        output='screen'
    )

    abs_teleop_node = Node(
        package='delcomp_cpp_pkg',
        executable='abs_teleop',
        name='abs_teleop_node',
        remappings=[
            ('viper_pose', '/inv3/pose')
        ],
        parameters=[{
            'teleop_param': 0.4
        }],
        output='screen'
    )

    delay_node = Node(
        package='delcomp_cpp_pkg',
        executable='delay_node',
        name='delay_node',
        remappings=[
            ('joy', '/inv3/buttons')
        ],
        parameters=[{
            'delay_time': 0
        }],
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        remappings=[
            ('joy', 'grasp_cmd')
        ],
        output='screen'
    )

    # Delay the start of dependent nodes by 3 seconds after ros2 pub exits
    delayed_launch = TimerAction(
        period=3.0,  # seconds
        actions=[abs_teleop_node, delay_node]
    )

    return LaunchDescription([
        inv3_node,
        send_use_for_sim_msg,
        RegisterEventHandler(
            OnProcessExit(
                target_action=send_use_for_sim_msg,
                on_exit=[delayed_launch]
            )
        ),
        joy_node  # independent
    ])
