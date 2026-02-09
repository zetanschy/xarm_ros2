#!/usr/bin/env python3
"""
MoveIt utility functions for xArm robot manipulation.

This module provides common functions used across multiple scripts
for MoveIt configuration, pose planning, and gripper control.
"""

import rclpy
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPlanningScene
import numpy as np

# Common constants
GROUP = "xarm6"
LINK = "link6"

# Gripper positions
GRIPPER_CLOSE_POS = 0.5
GRIPPER_OPEN_POS = 0.0


def setup_moveit_config(node, dof=6, add_gripper=True):
    """
    Setup MoveIt configuration for xArm robot.
    
    Args:
        node: ROS 2 node instance
        dof: Degrees of freedom (default: 6)
        add_gripper: Whether to include gripper (default: True)
    
    Returns:
        tuple: (moveit_config_dict, use_sim_time)
    """
    if node.has_parameter('use_sim_time'):
        use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
    else:
        node.declare_parameter('use_sim_time', False)
        use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
    
    node.get_logger().info(f"use_sim_time: {use_sim_time}")
    
    moveit_config_builder = MoveItConfigsBuilder(
        context=None,
        controllers_name='fake_controllers',
        dof=dof,
        robot_type='xarm',
        prefix='',
        limited=True,
        add_gripper=add_gripper,
    )
    moveit_config_builder.moveit_cpp(
        file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_planning_python.yaml"
    )
    moveit_config_dict = moveit_config_builder.to_dict()
    moveit_config_dict['use_sim_time'] = use_sim_time
    MoveItConfigsBuilder._add_qos_overrides_for_sim_time(moveit_config_dict)
    
    return moveit_config_dict, use_sim_time


def pose_to_stamped(position, orientation=None):
    """
    Convert position [x, y, z] to PoseStamped with optional orientation.
    
    Args:
        position: List [x, y, z] in meters
        orientation: Optional quaternion [x, y, z, w]. If None, uses downward orientation.
    
    Returns:
        PoseStamped: ROS 2 pose message
    """
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"
    pose_goal.pose.position.x = float(position[0])
    pose_goal.pose.position.y = float(position[1])
    pose_goal.pose.position.z = float(position[2])
    
    if orientation is not None:
        # Use provided orientation
        pose_goal.pose.orientation.x = float(orientation[0])
        pose_goal.pose.orientation.y = float(orientation[1])
        pose_goal.pose.orientation.z = float(orientation[2])
        pose_goal.pose.orientation.w = float(orientation[3])
    else:
        # Default orientation pointing down (gripper facing down)
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 1.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.0
    
    return pose_goal


def rotation_matrix_to_quaternion(rotation_matrix):
    """
    Convert 3x3 rotation matrix to quaternion [x, y, z, w].
    
    Args:
        rotation_matrix: 3x3 numpy array rotation matrix
    
    Returns:
        list: Quaternion [x, y, z, w]
    """
    trace = np.trace(rotation_matrix)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
    else:
        i = np.argmax([rotation_matrix[0, 0], rotation_matrix[1, 1], rotation_matrix[2, 2]])
        j = (i + 1) % 3
        k = (i + 2) % 3
        s = np.sqrt(rotation_matrix[i, i] - rotation_matrix[j, j] - rotation_matrix[k, k] + 1.0) * 2
        q = np.zeros(4)
        q[i] = 0.25 * s
        q[3] = (rotation_matrix[k, j] - rotation_matrix[j, k]) / s
        q[j] = (rotation_matrix[j, i] + rotation_matrix[i, j]) / s
        q[k] = (rotation_matrix[k, i] + rotation_matrix[i, k]) / s
        w, x, y, z = q[3], q[0], q[1], q[2]
    
    return [x, y, z, w]


def get_current_pose(xarm_moveit, link=LINK):
    """
    Get current end effector position and orientation.
    
    Args:
        xarm_moveit: MoveItPy instance
        link: Link name (default: LINK)
    
    Returns:
        tuple: (position [x, y, z], orientation [x, y, z, w])
    """
    planning_scene_monitor = xarm_moveit.get_planning_scene_monitor()
    with planning_scene_monitor.read_only() as scene:
        robot_state = scene.current_state
        transform = robot_state.get_global_link_transform(link)
        position = [transform[0, 3], transform[1, 3], transform[2, 3]]
        rotation_matrix = transform[:3, :3]
        orientation = rotation_matrix_to_quaternion(rotation_matrix)
    
    return position, orientation


def move_to_pose(planning_component, xarm_moveit, group, position, link=LINK, orientation=None):
    """
    Move arm to specified position with optional orientation.
    
    Args:
        planning_component: MoveIt planning component
        xarm_moveit: MoveItPy instance
        group: Planning group name
        position: List [x, y, z] in meters
        link: End effector link name (default: LINK)
        orientation: Optional quaternion [x, y, z, w]. If None, uses default downward orientation.
    
    Returns:
        bool: True if successful, False otherwise
    """
    planning_component.set_start_state_to_current_state()
    pose_goal = pose_to_stamped(position, orientation)
    planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=link)
    plan_result = planning_component.plan()
    
    if plan_result:
        xarm_moveit.execute(group, plan_result.trajectory, blocking=True)
        return True
    return False


def set_gripper_state(xarm_moveit, state_name):
    """
    Set gripper to named state (e.g., "open", "close").
    
    Args:
        xarm_moveit: MoveItPy instance
        state_name: Name of the gripper state
    """
    gripper = xarm_moveit.get_planning_component("xarm_gripper")
    gripper.set_start_state_to_current_state()
    gripper.set_goal_state(configuration_name=state_name)
    plan_result = gripper.plan()
    if plan_result:
        xarm_moveit.execute("xarm_gripper", plan_result.trajectory, blocking=True)


def set_gripper(node, gripper_client, position_rad):
    """
    Set gripper to specific position using action client.
    
    Args:
        node: ROS 2 node instance
        gripper_client: ActionClient for gripper control
        position_rad: Gripper position in radians
    """
    from rclpy.time import Time
    
    goal_msg = FollowJointTrajectory.Goal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['drive_joint']
    
    # For simulation, use Time(0) to let the controller use current time when received
    # For real hardware, use current time
    # Check if using sim time
    use_sim_time = False
    if node.has_parameter('use_sim_time'):
        use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
    
    if use_sim_time:
        # Use Time(0) for simulation - controller will use current sim time when received
        trajectory.header.stamp = Time(seconds=0, nanoseconds=0).to_msg()
    else:
        # Use current time for real hardware
        trajectory.header.stamp = node.get_clock().now().to_msg()
    
    point = JointTrajectoryPoint()
    point.positions = [float(position_rad)]
    point.velocities = [0.0]
    point.accelerations = [0.0]
    point.time_from_start.sec = 1  # Reduced from 2 to 1 second
    point.time_from_start.nanosec = 0
    trajectory.points = [point]
    goal_msg.trajectory = trajectory
    
    tolerance = JointTolerance()
    tolerance.name = 'drive_joint'
    tolerance.position = 0.1
    goal_msg.path_tolerance = [tolerance]
    goal_msg.goal_tolerance = [tolerance]
    goal_msg.goal_time_tolerance.sec = 1
    
    send_future = gripper_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    goal_handle = send_future.result()
    
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)

