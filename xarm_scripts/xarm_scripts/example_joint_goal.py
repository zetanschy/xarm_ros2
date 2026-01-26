#!/usr/bin/env python3
"""
Simple MoveIt2 Python API test script for xArm.

Based on the official MoveIt2 Python API example.

IMPORTANT: The launch file must include these planning_scene_monitor_parameters:
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,  # Required!
        "publish_robot_description_semantic": True,  # Required!
    }
    
These are not set by default and are needed for MoveItPy to find the robot description.
"""

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit.core.kinematic_constraints import construct_joint_constraint
import traceback
import time

def main():
    rclpy.init()
    node = Node("moveit_py_joint_goal")
    logger = node.get_logger()
    
    try:
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', False)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        logger.info("MoveIt2 Python API Test for xArm")
        logger.info(f"use_sim_time: {use_sim_time}")
        
        # Configuration
        planning_group = "xarm6"  # Change to "xarm7", "xarm5", etc. as needed
        logger.info(f"Initializing MoveItPy for planning group: {planning_group}")
        
        # Use the custom MoveItConfigsBuilder from uf_ros_lib (designed for xarm robots)
        # This builder automatically handles URDF/SRDF path resolution
        moveit_config_builder = MoveItConfigsBuilder(
            context=None,  # No launch context needed for standalone script
            controllers_name='fake_controllers',  # Use fake controllers for testing
            dof=6,  # xarm6 has 6 DOF
            robot_type='xarm',
            prefix='',
            limited=True,
        )
        moveit_config_builder.moveit_cpp(
            file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_planning_python.yaml"
        )
        # Get config dict
        moveit_config_dict = moveit_config_builder.to_dict()
        
        # Add use_sim_time to config dict for MoveItPy's internal node if not already present
        if 'use_sim_time' not in moveit_config_dict:
            moveit_config_dict['use_sim_time'] = use_sim_time
        
        # Apply QoS override fix (needed after adding use_sim_time)
        MoveItConfigsBuilder._add_qos_overrides_for_sim_time(moveit_config_dict)
        
        # Initialize MoveItPy
        xarm_moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
        xarm_arm = xarm_moveit.get_planning_component(planning_group)
        logger.info("✓ MoveItPy initialized")
        
        # Create a robot state to set the joint values
        robot_model = xarm_moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        
        # Set plan start state to current state
        xarm_arm.set_start_state_to_current_state()
        
        # Set target joint values (in radians)
        joint_values = {
            "joint1": 0.5,
            "joint2": 0.0,
            "joint3": -0.5,
            "joint4": 0.0,
            "joint5": -0.5,
            "joint6": 0.0,
        }
        
        robot_state.joint_positions = joint_values

        # Set the joint values as the goal state
        joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
                joint_model_group=xarm_moveit.get_robot_model().get_joint_model_group(planning_group),
        )
        xarm_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # Create a plan to the target pose
        logger.info("Planning to target joint configuration...")
        plan_result = xarm_arm.plan()
        if not plan_result:
            logger.error("Planning failed!")
            return 1
        
        robot_trajectory = plan_result.trajectory
        logger.info("Executing trajectory to target position...")
        xarm_moveit.execute(planning_group, robot_trajectory, blocking=True)
        logger.info("✓ Reached target position")
        
        time.sleep(5)
        
        # Return to home position using named configuration from SRDF
        logger.info("\nReturning to home position (using SRDF named configuration)...")
        xarm_arm.set_start_state_to_current_state()
        
        # Check available named target states
        named_targets = xarm_arm.named_target_states
        logger.info(f"Available named target states: {named_targets}")
        
        # Use "home" configuration from SRDF
        logger.info("Using 'home' configuration from SRDF")
        xarm_arm.set_goal_state(configuration_name="home")
        
        
        logger.info("Planning to home position...")
        home_plan_result = xarm_arm.plan()
        if not home_plan_result:
            logger.warn("Planning to home position failed!")
            return 1
        
        home_trajectory = home_plan_result.trajectory
        logger.info("Executing trajectory to home position...")
        xarm_moveit.execute(planning_group, home_trajectory, blocking=True)
        logger.info("✓ Returned to home position")

    except Exception as e:
        logger.error(f"✗ Error: {str(e)}")
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    import sys
    sys.exit(main())
