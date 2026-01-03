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

def main():
    rclpy.init()
    node = Node("test_simple_api")
    logger = node.get_logger()
    
    try:
        # Get use_sim_time parameter
        # When passed via --ros-args -p use_sim_time:=true, ROS2 automatically declares it
        # Try to get it first, and only declare if it doesn't exist
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
            file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_cpp.yaml"
        )
        moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()
        
        # Add use_sim_time to config dict for MoveItPy's internal node
        if 'use_sim_time' not in moveit_config_dict:
            moveit_config_dict['use_sim_time'] = use_sim_time
        
        # Fix for MoveItPy QoS override issue when use_sim_time is True
        # MoveItPy tries to set QoS overrides after node creation, which fails.
        # We must set them in the config dict before initialization.
        if use_sim_time:
            if 'qos_overrides' not in moveit_config_dict:
                moveit_config_dict['qos_overrides'] = {}
            if '/clock' not in moveit_config_dict['qos_overrides']:
                moveit_config_dict['qos_overrides']['/clock'] = {}
            if 'subscription' not in moveit_config_dict['qos_overrides']['/clock']:
                moveit_config_dict['qos_overrides']['/clock']['subscription'] = {}
            
            # Set all required QoS parameters for /clock subscription
            moveit_config_dict['qos_overrides']['/clock']['subscription']['durability'] = 'transient_local'
            moveit_config_dict['qos_overrides']['/clock']['subscription']['reliability'] = 'reliable'
            moveit_config_dict['qos_overrides']['/clock']['subscription']['history'] = 'keep_last'
            moveit_config_dict['qos_overrides']['/clock']['subscription']['depth'] = 10
        
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
        # Adjust these values based on your robot's joint limits
        joint_values = {
            "joint1": 0.5,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
            "joint5": 0.0,
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
        plan_result = xarm_arm.plan()
        robot_trajectory = plan_result.trajectory
        xarm_moveit.execute(planning_group,robot_trajectory, blocking=True)
        

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
