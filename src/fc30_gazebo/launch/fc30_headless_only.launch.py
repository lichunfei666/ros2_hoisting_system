#!/usr/bin/env python3
"""
FC30 Drone Headless Simulation Launch File
This launch file only runs the essential nodes without any Gazebo GUI or rendering
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    return LaunchDescription([
        # FC30 Control Node
        Node(
            package='fc30_gazebo',
            executable='fc30_controller',
            name='fc30_controller',
            output='screen',
            parameters=[],
        ),
        
        # PSDK Bridge Node
        Node(
            package='fc30_gazebo',
            executable='psdk_bridge',
            name='psdk_bridge',
            output='screen',
            parameters=[],
        ),
        
        # State Estimator Node
        Node(
            package='fc30_gazebo',
            executable='state_estimator',
            name='state_estimator',
            output='screen',
            parameters=[],
        ),
        
        # Trajectory Planner Node
        Node(
            package='fc30_gazebo',
            executable='trajectory_planner',
            name='trajectory_planner',
            output='screen',
            parameters=[],
        ),
    ])