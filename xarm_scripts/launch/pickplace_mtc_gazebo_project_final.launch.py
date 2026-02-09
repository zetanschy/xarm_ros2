#!/usr/bin/env python3
"""
Launch file for pick-and-place to trash bin using Gemini Vision + color_sorter functions.

Prerequisites – launch Gazebo first in another terminal:
  ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py add_gripper:=true

Then run this:
  ros2 launch xarm_scripts pickplace_mtc_gazebo_project_final.launch.py
"""

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)

    # Optional fallback pick coordinates (if Gemini fails)
    pick_x = LaunchConfiguration('pick_x')
    pick_y = LaunchConfiguration('pick_y')

    xarm_type = '{}{}'.format(
        robot_type.perform(context),
        dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '',
    )

    # ── ros2_controllers config (needed by MoveItConfigsBuilder) ──
    ros2_controllers_path = generate_ros2_control_params_temp_file(
        os.path.join(
            get_package_share_directory('xarm_controller'),
            'config', '{}_controllers.yaml'.format(xarm_type),
        ),
        prefix=prefix.perform(context),
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        robot_type=robot_type.perform(context),
    )

    # ── MoveIt config (URDF, SRDF, kinematics, OMPL) ──
    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
        ros2_control_params=ros2_controllers_path,
    ).planning_pipelines(
        pipelines=['ompl'],
        default_planning_pipeline='ompl',
    ).to_moveit_configs()

    moveit_config_dict = moveit_config.to_dict()

    # ── Pick and place node (uses color_sorter functions, not MTC) ──
    pickplace_node = Node(
        package='xarm_scripts',
        executable='pickplace_project_final',
        output='screen',
        parameters=[
            moveit_config_dict,
            {
                'use_sim_time': True,
                'pick_x': float(pick_x.perform(context)),  # Fallback if Gemini fails
                'pick_y': float(pick_y.perform(context)),  # Fallback if Gemini fails
            },
        ],
    )

    return [pickplace_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('robot_type', default_value='xarm'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('add_gripper', default_value='true'),
        DeclareLaunchArgument('pick_x', default_value='-0.3',
                              description='Fallback X position if Gemini vision fails'),
        DeclareLaunchArgument('pick_y', default_value='-0.4',
                              description='Fallback Y position if Gemini vision fails'),
        OpaqueFunction(function=launch_setup),
    ])

