#!/usr/bin/env python3
"""
Launch file for example_kinematic_constraints script.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Using or not time from simulation",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    example_kinematic_constraints_node = Node(
        package='xarm_scripts',
        executable='example_kinematic_constraints',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        example_kinematic_constraints_node,
    ])

