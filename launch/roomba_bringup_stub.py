"""Roomba bringup launch (stub mode, no hardware required).

Starts roomba_node with use_stub=true and monitor_node.
Sensor data will not be published (stub Read returns nothing),
but drive commands from the operation source are processed and logged.

    # Terminal 1: bringup (stub)
    bazel run //launch:roomba_bringup_stub

    # Terminal 2: operation source (choose one)
    bazel run //keyboard_node:keyboard_node   # keyboard teleoperation
"""
import os

import launch
import launch_ros.actions


def generate_launch_description():
    """Launch roomba_node (stub) and monitor_node."""
    params_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'roomba_params.yaml')

    stub_overrides = {'use_stub': True}

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable='roomba_node/roomba_node',
            output='log',
            name='roomba_node',
            parameters=[params_file, stub_overrides],
        ),
        launch_ros.actions.Node(
            executable='monitor_node/monitor_node',
            output='screen',
            name='monitor_node',
            parameters=[params_file],
        ),
    ])
