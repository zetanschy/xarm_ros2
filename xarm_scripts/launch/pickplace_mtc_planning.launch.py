#!/usr/bin/env python3
"""
Self-contained launch file for the MTC pick-and-place demo on xarm6.

Launches everything needed:
  - robot_state_publisher (TF from URDF)
  - static_transform_publisher (world → link_base)
  - ros2_control_node (fake hardware)
  - controller spawners (joint_state_broadcaster, arm & gripper controllers)
  - move_group (with ExecuteTaskSolutionCapability)
  - RViz with MTC "Motion Planning Tasks" display
  - MTC pickplace_mtc node

Usage:
  ros2 launch xarm_scripts pickplace_mtc.launch.py
  ros2 launch xarm_scripts pickplace_mtc.launch.py add_gripper:=true dof:=6
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

    xarm_type = '{}{}'.format(
        robot_type.perform(context),
        dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '',
    )

    # ── ros2_controllers config ──
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

    # ── MoveIt config (URDF, SRDF, kinematics, OMPL, controllers) ──
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
    ).to_moveit_configs()

    moveit_config_dict = moveit_config.to_dict()

    # ── move_group node (with MTC ExecuteTaskSolution capability) ──
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            {'capabilities': 'move_group/ExecuteTaskSolutionCapability'},
        ],
    )

    # ── RViz with MTC display ──
    rviz_config_file = os.path.join(
        get_package_share_directory('xarm_scripts'), 'rviz', 'mtc.rviz',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # ── Static TF: world → link_base ──
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'world', '{}link_base'.format(prefix.perform(context)),
        ],
    )

    # ── Robot state publisher (URDF TF) ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # ── ros2_control (fake hardware) ──
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output='screen',
    )

    # ── Controller spawners ──
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
    )

    controllers = ['{}{}_traj_controller'.format(prefix.perform(context), xarm_type)]
    if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append(
            '{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context))
        )
    elif add_bio_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append(
            '{}bio_gripper_traj_controller'.format(prefix.perform(context))
        )

    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
            ],
        ))

    # ── MTC pickplace node ──
    mtc_node = Node(
        package='xarm_scripts',
        executable='pickplace_mtc_planning',
        output='screen',
        parameters=[
            moveit_config_dict,
        ],
    )

    return [
        robot_state_publisher,
        static_tf,
        ros2_control_node,
        joint_state_broadcaster,
        move_group_node,
        rviz_node,
        mtc_node,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('robot_type', default_value='xarm'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('add_gripper', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
