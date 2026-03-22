"""Roomba bringup launch (stub mode, no hardware required).

Starts roomba_node with use_stub=true and monitor_node.
Sensor data will not be published (stub Read returns nothing),
but drive commands from the operation source are processed and logged.

    # Terminal 1: bringup (stub)
    bazel run //launch:roomba_bringup_stub

    # Terminal 2: keyboard control + mode switching
    bazel run //keyboard_node:keyboard_node   # Tab to toggle MANUAL / WALL_FOLLOW
"""
import os

import launch
import launch_ros.actions


def generate_launch_description():
    """Launch roomba_node (stub), drive_mux, and monitor_node."""
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
            executable='drive_mux_node/drive_mux_node',
            output='log',
            name='drive_mux',
        ),
        launch_ros.actions.Node(
            executable='planner_node/planner_node',
            output='log',
            name='planner_node',
            parameters=[params_file],
        ),
        launch_ros.actions.Node(
            executable='monitor_node/monitor_node',
            output='screen',
            name='monitor_node',
            parameters=[params_file],
        ),
    ])
