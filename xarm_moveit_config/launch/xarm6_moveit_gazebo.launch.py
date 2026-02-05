#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    # World selection argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='table.world',
        description='World file to load (e.g., table.world, table_gz.world, table_color_objects.world)'
    )
    
    # Robot position arguments
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='-0.2',
        description='Initial X position of the robot'
    )
    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='-0.5',
        description='Initial Y position of the robot'
    )
    robot_z_arg = DeclareLaunchArgument(
        'robot_z',
        default_value='1.021',
        description='Initial Z position of the robot'
    )
    robot_yaw_arg = DeclareLaunchArgument(
        'robot_yaw',
        default_value='1.571',
        description='Initial Yaw (rotation around Z) of the robot in radians'
    )
    
    # Depth camera argument
    enable_depth_camera_arg = DeclareLaunchArgument(
        'enable_depth_camera',
        default_value='false',
        description='Enable depth camera and octomap for collision avoidance (requires overhead camera)'
    )
    
    world = LaunchConfiguration('world')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    enable_depth_camera = LaunchConfiguration('enable_depth_camera')
    
    # robot moveit gazebo launch
    # xarm_moveit_config/launch/_robot_moveit_gazebo.launch.py
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_gazebo.launch.py'])),
        launch_arguments={
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
            'world': world,
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_z': robot_z,
            'robot_yaw': robot_yaw,
            'enable_depth_camera': enable_depth_camera,
        }.items(),
    )
    
    return LaunchDescription([
        world_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_yaw_arg,
        enable_depth_camera_arg,
        robot_moveit_gazebo_launch
    ])
