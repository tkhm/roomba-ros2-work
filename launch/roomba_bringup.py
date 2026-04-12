"""Roomba bringup launch (real hardware).

Starts roomba_node (real serial) and monitor_node (terminal dashboard).
An operation source must be started separately depending on the use case:

    # Terminal 1: bringup (always the same)
    bazel run //launch:roomba_bringup

    # Terminal 2: keyboard control + mode switching
    bazel run //keyboard_node:keyboard_node   # Tab to toggle MANUAL / WALL_FOLLOW
"""
import os

import launch
import launch_ros.actions


def generate_launch_description():
    """Launch roomba_node, drive_mux, and monitor_node (real hardware)."""
    params_file = os.path.join(os.path.dirname(__file__), '..', 'config', 'roomba_params.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable='roomba_node/roomba_node',
            output='log',
            name='roomba_node',
            parameters=[params_file],
        ),
        launch_ros.actions.Node(
            executable='tof_node/tof_node',
            output='log',
            name='tof_node',
            parameters=[params_file],
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
        # monitor_node uses screen output to render the terminal dashboard
        launch_ros.actions.Node(
            executable='monitor_node/monitor_node',
            output='screen',
            name='monitor_node',
            parameters=[params_file],
        ),
    ])
