"""Roomba bringup launch (real hardware).

Starts roomba_node (real serial) and monitor_node (terminal dashboard).
An operation source must be started separately depending on the use case:

    # Terminal 1: bringup
    bazel run //launch:roomba_bringup

    # Terminal 2: operation source (choose one)
    bazel run //keyboard_node:keyboard_node   # keyboard teleoperation
"""
import os

import launch
import launch_ros.actions


def generate_launch_description():
    """Launch roomba_node (real) and monitor_node."""
    params_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'roomba_params.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable='roomba_node/roomba_node',
            output='log',
            name='roomba_node',
            parameters=[params_file],
        ),
        # monitor_node uses screen output to render the terminal dashboard
        launch_ros.actions.Node(
            executable='monitor_node/monitor_node',
            output='screen',
            name='monitor_node',
            parameters=[params_file],
        ),
    ])
