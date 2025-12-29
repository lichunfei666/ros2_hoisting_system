#!/usr/bin/env python3
"""
FC30 Drone Gazebo Simulation Launch File
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fc30_gazebo')
    
    # Path to the world file with FC30 drone model
    world_file = os.path.join(pkg_dir, 'worlds', 'fc30_drone_world.sdf')
    
    # Set environment variables to disable all rendering
    os.environ['GAZEBO_RENDERING_ENABLED'] = '0'
    os.environ['GAZEBO_VISUALIZATION_ENABLED'] = '0'
    os.environ['OGRE_RENDERING_ENABLED'] = '0'
    
    # Set Gazebo model paths for both Gazebo Classic and Gazebo Sim
    gazebo_model_path = os.path.join(pkg_dir, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path += ':' + os.environ['GAZEBO_MODEL_PATH']
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path
    
    # For Gazebo Sim (gz sim)
    gz_sim_resource_path = os.path.join(pkg_dir, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_sim_resource_path += ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_sim_resource_path
    
    # Load URDF file for robot model
    urdf_file = os.path.join(pkg_dir, 'models', 'fc30', 'robot.urdf')
    
    # Robot description command
    robot_description = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        # Launch Gazebo in pure server mode with no rendering
        ExecuteProcess(
            cmd=['gz', 'sim', '--headless-rendering', world_file],
            output='screen',
            name='gazebo'
        ),
        
        # FC30 Control Node (placeholder for actual controller)
        Node(
            package='fc30_gazebo',
            executable='fc30_controller',
            name='fc30_controller',
            output='screen',
            parameters=[],
        ),
        
        # ROS 2 - PSDK Bridge Node
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
        
        # Robot State Publisher - publishes robot model and TF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'fc30_rviz_config.rviz')],
        ),
    ])