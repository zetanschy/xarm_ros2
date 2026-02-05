#!/usr/bin/env python3
"""
Color Sorting Launch File

This launch file starts the color detector and color sorter nodes.

Usage Examples:

1. Auto mode - pick all detected colors:
   ros2 launch xarm_scripts color_sorting.launch.py auto_mode:=true

2. Target a specific color (R/G/B):
   ros2 launch xarm_scripts color_sorting.launch.py target_color:=R

3. Interactive mode (run sorter separately for user input):
   Terminal 1: ros2 launch xarm_scripts color_sorting.launch.py sorter_only:=false
   Terminal 2: ros2 run xarm_scripts color_sorter --ros-args -p use_sim_time:=true

Note: Make sure the Gazebo simulation with color_objects world is running first!
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    target_color_arg = DeclareLaunchArgument(
        'target_color',
        default_value='',
        description='Target color to pick (R/G/B). Empty for interactive/auto mode'
    )
    
    auto_mode_arg = DeclareLaunchArgument(
        'auto_mode',
        default_value='false',
        description='Auto mode: automatically pick all detected colors'
    )
    
    run_sorter_arg = DeclareLaunchArgument(
        'run_sorter',
        default_value='true',
        description='Whether to run the color sorter (set false to run separately)'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_color = LaunchConfiguration('target_color')
    auto_mode = LaunchConfiguration('auto_mode')
    run_sorter = LaunchConfiguration('run_sorter')
    
    # Color detector node
    color_detector_node = Node(
        package='xarm_scripts',
        executable='color_detector',
        name='color_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Color sorter node - delayed start to allow detector to initialize
    color_sorter_node = TimerAction(
        period=5.0,  # Wait 5 seconds before starting sorter
        actions=[
            Node(
                package='xarm_scripts',
                executable='color_sorter',
                name='color_sorter',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'target_color': target_color,
                    'auto_mode': auto_mode,
                }],
                condition=IfCondition(run_sorter)
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        target_color_arg,
        auto_mode_arg,
        run_sorter_arg,
        color_detector_node,
        color_sorter_node,
    ])

