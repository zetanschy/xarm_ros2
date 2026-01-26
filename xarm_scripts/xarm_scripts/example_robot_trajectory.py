#!/usr/bin/env python3
"""
Example demonstrating Robot Trajectory manipulation in MoveIt 2 Python API.

This example shows how to:
- Get trajectory from planning
- Inspect trajectory waypoints
- Modify trajectory
- Execute trajectory
- Work with trajectory timing

Reference: https://moveit.ai/moveit/ros/python/google/2023/02/15/MoveIt-Humble-Release.html
"""

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit.core.kinematic_constraints import construct_joint_constraint
import numpy as np
import traceback


def main():
    rclpy.init()
    node = Node("example_robot_trajectory")
    logger = node.get_logger()
    
    try:
        # Get use_sim_time parameter
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', False)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        logger.info("="*60)
        logger.info("MoveIt 2 Python API: Robot Trajectory Example")
        logger.info("="*60)
        
        # Configuration
        planning_group = "xarm6"
        logger.info(f"Initializing MoveItPy for planning group: {planning_group}")
        
        # Build MoveIt configuration
        moveit_config_builder = MoveItConfigsBuilder(
            context=None,
            controllers_name='fake_controllers',
            dof=6,
            robot_type='xarm',
            prefix='',
            limited=True,
        )
        moveit_config_builder.moveit_cpp(
            file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_planning_python.yaml"
        )
        moveit_config_dict = moveit_config_builder.to_dict()
        
        if 'use_sim_time' not in moveit_config_dict:
            moveit_config_dict['use_sim_time'] = use_sim_time
        
        MoveItConfigsBuilder._add_qos_overrides_for_sim_time(moveit_config_dict)
        
        # Initialize MoveItPy
        xarm_moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
        robot_model = xarm_moveit.get_robot_model()
        planning_component = xarm_moveit.get_planning_component(planning_group)
        joint_model_group = robot_model.get_joint_model_group(planning_group)
        logger.info("MoveItPy initialized successfully")
        
        # Set start state to current state
        planning_component.set_start_state_to_current_state()
        
        # Create a goal state
        logger.info("\n--- Creating Goal State and Planning ---")
        robot_state = RobotState(robot_model)
        joint_values = {
            "joint1": 0.5,
            "joint2": 0.0,
            "joint3": -0.5,
            "joint4": 0.0,
            "joint5": -0.5,
            "joint6": 0.0,
        }
        robot_state.joint_positions = joint_values
        robot_state.update()
        
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=joint_model_group,
        )
        planning_component.set_goal_state(motion_plan_constraints=[joint_constraint])
        
        # Plan trajectory
        logger.info("Planning trajectory...")
        plan_result = planning_component.plan()
        
        if not plan_result:
            logger.error("Planning failed!")
            return 1
        
        # Get trajectory
        logger.info("\n--- Inspecting Trajectory ---")
        robot_trajectory = plan_result.trajectory
        
        # Get trajectory as ROS message for timing information
        trajectory_msg = robot_trajectory.get_robot_trajectory_msg()
        num_waypoints_total = len(trajectory_msg.joint_trajectory.points)
        
        # Get trajectory information using the duration property
        logger.info(f"Trajectory duration: {robot_trajectory.duration:.4f} seconds")
        logger.info(f"Number of waypoints: {num_waypoints_total}")
        
        # Inspect waypoints
        logger.info("\n--- Trajectory Waypoints ---")
        # Get joint names from the trajectory message
        joint_names = trajectory_msg.joint_trajectory.joint_names
        if not joint_names:
            joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        num_waypoints_to_show = min(5, num_waypoints_total)
        logger.info(f"Showing first {num_waypoints_to_show} waypoints:")
        
        for i in range(num_waypoints_to_show):
            if i < len(trajectory_msg.joint_trajectory.points):
                point = trajectory_msg.joint_trajectory.points[i]
                time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                logger.info(f"\nWaypoint {i+1} (t={time_from_start:.4f}s):")
                # Get joint positions from the trajectory message point
                for j, joint_name in enumerate(joint_names):
                    if j < len(point.positions):
                        joint_value = point.positions[j]
                        logger.info(f"  {joint_name}: {joint_value:.4f} rad")
        
        if num_waypoints_total > num_waypoints_to_show:
            logger.info(f"... and {num_waypoints_total - num_waypoints_to_show} more waypoints")
        
        # Trajectory message already obtained above
        logger.info("\n--- Trajectory as ROS Message ---")
        logger.info(f"Trajectory message type: {type(trajectory_msg)}")
        logger.info(f"Joint trajectory has {len(trajectory_msg.joint_trajectory.points)} points")
        
        # Inspect trajectory timing
        logger.info("\n--- Trajectory Timing Analysis ---")
        if len(trajectory_msg.joint_trajectory.points) > 0:
            first_point = trajectory_msg.joint_trajectory.points[0]
            last_point = trajectory_msg.joint_trajectory.points[-1]
            
            first_time = first_point.time_from_start.sec + first_point.time_from_start.nanosec * 1e-9
            last_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
            
            logger.info(f"First waypoint time: {first_time:.4f}s")
            logger.info(f"Last waypoint time: {last_time:.4f}s")
            logger.info(f"Total trajectory time: {last_time - first_time:.4f}s")
        
        # Get end effector path
        logger.info("\n--- End Effector Path ---")
        end_effector_link = "link6"
        logger.info(f"Computing end effector path for {end_effector_link}...")
        
        path_points = []
        # Create robot states from trajectory message points and compute transforms
        for i in range(num_waypoints_total):
            if i < len(trajectory_msg.joint_trajectory.points):
                point = trajectory_msg.joint_trajectory.points[i]
                # Create a robot state with the joint positions from this waypoint
                waypoint_state = RobotState(robot_model)
                waypoint_state.joint_positions = dict(zip(joint_names, point.positions))
                waypoint_state.update()
                
                # Get transform for end effector
                transform = waypoint_state.get_global_link_transform(end_effector_link)
                # Transform is a 4x4 numpy array, translation is in [:3, 3]
                pos = transform[:3, 3]
                path_points.append([pos[0], pos[1], pos[2]])
        
        logger.info(f"End effector path has {len(path_points)} points")
        if len(path_points) > 0:
            logger.info(f"Start position: [{path_points[0][0]:.4f}, {path_points[0][1]:.4f}, {path_points[0][2]:.4f}]")
            logger.info(f"End position: [{path_points[-1][0]:.4f}, {path_points[-1][1]:.4f}, {path_points[-1][2]:.4f}]")
    
        xarm_moveit.execute(planning_group, robot_trajectory, blocking=True)
        logger.info("Trajectory executed successfully!")
        
        logger.info("\n" + "="*60)
        logger.info("Robot Trajectory example completed!")
        logger.info("="*60)

    except Exception as e:
        logger.error(f"Error: {str(e)}")
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    import sys
    sys.exit(main())

