"""Launch turtlesim and the currently implemented relay member."""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )
    member_01 = Node(
        package='digit_relay',
        executable='digit_relay_member_01',
        name='member_01',
        output='screen',
    )
    # Member 01 owns the final turn in this single-member demonstration.
    shutdown_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=member_01,
            on_exit=[Shutdown(reason='Distributed digit relay completed.')],
        )
    )

    return LaunchDescription([
        turtlesim,
        member_01,
        shutdown_when_complete,
    ])
