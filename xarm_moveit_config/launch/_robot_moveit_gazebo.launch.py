#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    
    # World and robot position parameters
    world = LaunchConfiguration('world', default='table.world')
    robot_x = LaunchConfiguration('robot_x', default='-0.2')
    robot_y = LaunchConfiguration('robot_y', default='-0.5')
    robot_z = LaunchConfiguration('robot_z', default='1.021')
    robot_yaw = LaunchConfiguration('robot_yaw', default='1.571')
    
    # Depth camera and octomap parameters
    enable_depth_camera = LaunchConfiguration('enable_depth_camera', default=False)
    
    # Get world filename and check if overhead camera should be enabled
    world_file = world.perform(context)
    add_overhead_camera = 'true' if 'color_objects' in world_file else 'false'
    # La camara de muneca se prende sola con el mundo del ArUco (Tarea 2), o a
    # mano con add_wrist_camera:=true si se quiere sobre cualquier otro mundo.
    add_wrist_camera = LaunchConfiguration(
        'add_wrist_camera',
        default='true' if 'aruco' in world_file else 'false').perform(context)
    
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    gz_type = LaunchConfiguration('gz_type', default='ign').perform(context)
    gz_type = 'ignition' if gz_type == 'ign' else gz_type

    ros2_control_plugin = 'gz_ros2_control/GazeboSimSystem' if gz_type == 'gz' else 'ign_ros2_control/IgnitionSystem' if gz_type == 'ignition' else 'gazebo_ros2_control/GazeboSystem'
    controllers_name = 'fake_controllers'

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type.perform(context)
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_overhead_camera=add_overhead_camera,
        add_wrist_camera=add_wrist_camera,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).planning_scene_monitor(
        publish_robot_description=True,  # Required for MoveItPy Python API
        publish_robot_description_semantic=True,  # Required for MoveItPy Python API
    )
    
    # Add sensors_3d configuration for octomap when depth camera is explicitly enabled
    enable_depth_value = enable_depth_camera.perform(context) if context else 'false'
    if enable_depth_value in ('True', 'true'):
        moveit_config = moveit_config.sensors_3d()
    
    moveit_config = moveit_config.to_moveit_configs()

    moveit_config_dump = yaml.dump(moveit_config.to_dict())

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common2.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'no_gui_ctrl': no_gui_ctrl,
            'show_rviz': 'false',
            'use_sim_time': 'true',
            'moveit_config_dump': moveit_config_dump,
        }.items(),
    )

    # robot gazebo launch
    # xarm_gazebo/launch/_robot_beside_table_gazebo.launch.py
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'launch', '_robot_beside_table_gazebo.launch.py'])),
        launch_arguments={
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'moveit_config_dump': moveit_config_dump,
            'load_controller': 'true',
            'show_rviz': 'true',
            'no_gui_ctrl': no_gui_ctrl,
            'gz_type': gz_type,
            'world': world,
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_z': robot_z,
            'robot_yaw': robot_yaw,
            'enable_depth_camera': enable_depth_camera,
        }.items(),
    )

    return [
        robot_gazebo_launch,
        robot_moveit_common_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
