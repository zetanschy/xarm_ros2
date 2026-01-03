#!/usr/bin/env python3
"""
Launch file for moveit_py_pose_goal script with use_sim_time parameter.

This launch file sets use_sim_time to True, which is required when running
with Gazebo simulation to ensure proper time synchronization between nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare launch argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Using or not time from simulation",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Node to run the moveit_py_pose_goal script
    # Note: use_sim_time must be set to True when running with Gazebo
    moveit_py_pose_goal_node = Node(
        package='xarm_scripts',
        executable='moveit_py_pose_goal',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        moveit_py_pose_goal_node,
    ])

