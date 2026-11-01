"""Launch turtlesim and all four relay members used on the test branch."""

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
    member_02 = Node(
            package='digit_relay',
            executable='digit_relay_member_02',
            name='member_02',
            output='screen',
    )
    member_03 = Node(
            package='digit_relay',
            executable='digit_relay_member_03',
            name='member_03',
            output='screen',
    )
    member_04 = Node(
            package='digit_relay',
            executable='digit_relay_member_04',
            name='member_04',
            output='screen',
    )
    member_05 = Node(
            package='digit_relay',
            executable='digit_relay_member_05',
            name='member_05',
            output='screen',
    )
    member_06 = Node(
                package='digit_relay',
                executable='digit_relay_member_06',
                name='member_06',
                output='screen',
    )
    member_07 = Node(
                package='digit_relay',
                executable='digit_relay_member_07',
                name='member_07',
                output='screen',
    )
    member_08 = Node(
                package='digit_relay',
                executable='digit_relay_member_08',
                name='member_08',
                output='screen',
    )
    member_09 = Node(
                package='digit_relay',
                executable='digit_relay_member_09',
                name='member_09',
                output='screen',
    )
    shutdown_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=member_09,
            on_exit=[Shutdown(reason='Distributed digit relay completed.')],
        )
    )
    return LaunchDescription([
        turtlesim,
        member_01,
        member_02,
        member_03,
        member_04,
        member_05,
        member_06,
        member_07,
        member_08,
        member_09,
        shutdown_when_complete,
    ])
