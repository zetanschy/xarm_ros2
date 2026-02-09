#!/usr/bin/env python3
"""
Unified launch file for xarm_scripts.

This launch file can run any script from xarm_scripts package by specifying
the script name as a parameter.

Usage examples:
  # Run example_joint_goal
  ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_joint_goal

  # Run example_pose_goal with use_sim_time
  ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_pose_goal use_sim_time:=true

  # Run test_pymoveit2_api
  ros2 launch xarm_scripts xarm_scripts.launch.py script:=test_pymoveit2_api use_sim_time:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


# Available scripts in xarm_scripts package (must match entry_points in setup.py)
AVAILABLE_SCRIPTS = [
    "example_joint_goal",
    "example_pose_goal",
    "example_robot_state",
    "example_robot_trajectory",
    "example_collision",
    "example_kinematic_constraints",
    "example_multi_pipeline",
    "example_planning_scene",
    "example_transforms",
    "example_advanced_planning",
    "test_pymoveit2_api",
    "pick_and_place",
    "pickplace_mtc",
]


def launch_setup(context, *args, **kwargs):
    """Setup function to create the node with the selected script."""
    script_name = context.launch_configurations.get('script', 'example_joint_goal')
    use_sim_time_str = context.launch_configurations.get('use_sim_time', 'true')
    
    # Convert string to boolean
    use_sim_time = use_sim_time_str.lower() in ('true', '1', 'yes', 'on')
    
    # Validate script name
    if script_name not in AVAILABLE_SCRIPTS:
        raise ValueError(
            f"Unknown script '{script_name}'. "
            f"Available scripts: {', '.join(AVAILABLE_SCRIPTS)}"
        )
    
    # Node to run the selected script
    xarm_script_node = Node(
        package='xarm_scripts',
        executable=script_name,  # Use the actual string value, not LaunchConfiguration
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,  # Now a boolean, not a string
        }],
    )
    
    return [xarm_script_node]


def generate_launch_description():
    # Declare launch arguments
    script_arg = DeclareLaunchArgument(
        "script",
        default_value="example_joint_goal",
        description=f"Script to run. Available: {', '.join(AVAILABLE_SCRIPTS)}",
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Using or not time from simulation",
    )
    
    return LaunchDescription([
        script_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup),
    ])

