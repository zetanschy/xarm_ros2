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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit.core.kinematic_constraints import construct_joint_constraint
import yaml
import json
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
        
        logger.info("="*60)
        logger.info("MoveIt2 Python API Test for xArm")
        logger.info(f"use_sim_time: {use_sim_time}")
        logger.info("="*60)
        
        # DEBUG: Check ROS2 environment
        logger.info("DEBUG: ROS2 Environment Check")
        logger.info(f"  Node name: {node.get_name()}")
        logger.info(f"  Node namespace: {node.get_namespace()}")
        
        # Check clock topic
        try:
            clock_topic = '/clock'
            logger.info(f"  Checking /clock topic availability...")
            # Create QoS profile for clock topic
            clock_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )
            logger.info(f"  Clock QoS profile configured: TRANSIENT_LOCAL, RELIABLE")
        except Exception as e:
            logger.warn(f"  Could not check clock topic: {e}")
            import traceback as tb
            logger.warn(f"  Traceback: {tb.format_exc()}")
        
        # Check node parameters
        logger.info("  Node parameters:")
        try:
            # Try to get common parameters
            common_params = ['use_sim_time']
            for param_name in common_params:
                try:
                    if node.has_parameter(param_name):
                        param_value = node.get_parameter(param_name).get_parameter_value()
                        logger.info(f"    {param_name}: {param_value}")
                except Exception as param_e:
                    logger.debug(f"    Could not get {param_name}: {param_e}")
        except Exception as e:
            logger.warn(f"  Could not list parameters: {e}")
        
        logger.info("="*60)
        
        # Configuration
        planning_group = "xarm6"  # Change to "xarm7", "xarm5", etc. as needed
        
        # Instantiate MoveItPy and get planning component
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
        
        # DEBUG: Log config dict structure
        logger.info("="*60)
        logger.info("DEBUG: MoveItPy Config Dict Analysis")
        logger.info("="*60)
        logger.info(f"Config dict keys: {list(moveit_config_dict.keys())}")
        
        # Check for use_sim_time
        if 'use_sim_time' in moveit_config_dict:
            logger.info(f"use_sim_time in config_dict: {moveit_config_dict['use_sim_time']}")
        else:
            logger.info("use_sim_time NOT in config_dict")
            logger.info("Adding use_sim_time to config_dict...")
            moveit_config_dict['use_sim_time'] = use_sim_time
            logger.info(f"Added use_sim_time: {moveit_config_dict['use_sim_time']}")
        
        # Check for qos_overrides
        if 'qos_overrides' in moveit_config_dict:
            logger.info(f"qos_overrides in config_dict: {moveit_config_dict['qos_overrides']}")
            if isinstance(moveit_config_dict['qos_overrides'], dict):
                logger.info(f"qos_overrides keys: {list(moveit_config_dict['qos_overrides'].keys())}")
                if '/clock' in moveit_config_dict['qos_overrides']:
                    logger.info(f"qos_overrides['/clock']: {moveit_config_dict['qos_overrides']['/clock']}")
        else:
            logger.info("qos_overrides NOT in config_dict")
        
        # CRITICAL FIX: When use_sim_time is True, we MUST set the QoS override for /clock
        # BEFORE MoveItPy initializes, otherwise MoveItPy will try to set it and fail.
        # The parameter format in ROS2 is: qos_overrides./clock.subscription.durability
        # But in the config dict, we use nested dictionary structure
        if use_sim_time:
            logger.info("="*60)
            logger.info("DEBUG: Setting QoS override for /clock topic (required for sim_time)")
            logger.info("="*60)
            
            if 'qos_overrides' not in moveit_config_dict:
                moveit_config_dict['qos_overrides'] = {}
            
            # ROS2 expects the parameter as: qos_overrides./clock.subscription.durability
            # In config dict, this translates to nested structure:
            # qos_overrides['/clock']['subscription']['durability']
            if '/clock' not in moveit_config_dict['qos_overrides']:
                moveit_config_dict['qos_overrides']['/clock'] = {}
            if 'subscription' not in moveit_config_dict['qos_overrides']['/clock']:
                moveit_config_dict['qos_overrides']['/clock']['subscription'] = {}
            
            # Set all required QoS parameters for /clock subscription
            # ROS2 uses string values for QoS policies
            # Durability: "volatile" or "transient_local"
            moveit_config_dict['qos_overrides']['/clock']['subscription']['durability'] = 'transient_local'
            logger.info("Set qos_overrides['/clock']['subscription']['durability'] = 'transient_local'")
            
            # Reliability: "best_effort" or "reliable"
            moveit_config_dict['qos_overrides']['/clock']['subscription']['reliability'] = 'reliable'
            logger.info("Set qos_overrides['/clock']['subscription']['reliability'] = 'reliable'")
            
            # History: "keep_last" or "keep_all"
            moveit_config_dict['qos_overrides']['/clock']['subscription']['history'] = 'keep_last'
            logger.info("Set qos_overrides['/clock']['subscription']['history'] = 'keep_last'")
            
            # Depth: integer value for keep_last history
            moveit_config_dict['qos_overrides']['/clock']['subscription']['depth'] = 10
            logger.info("Set qos_overrides['/clock']['subscription']['depth'] = 10")
            
            logger.info(f"Final qos_overrides structure: {moveit_config_dict['qos_overrides']}")
            logger.info("="*60)
        
        # Log a sample of the config dict (first few keys)
        logger.info("Sample config dict entries:")
        for key in list(moveit_config_dict.keys())[:10]:
            value = moveit_config_dict[key]
            if isinstance(value, dict):
                logger.info(f"  {key}: <dict with {len(value)} keys>")
            elif isinstance(value, list):
                logger.info(f"  {key}: <list with {len(value)} items>")
            else:
                logger.info(f"  {key}: {value}")
        
        logger.info("="*60)
        logger.info("Attempting MoveItPy initialization...")
        logger.info("="*60)
        
        try:
            xarm_moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
            logger.info("✓ MoveItPy initialized successfully")
            
            # DEBUG: Try to inspect the MoveItPy node (if possible)
            try:
                # MoveItPy creates an internal node, let's see if we can check its parameters
                logger.info("DEBUG: Checking if MoveItPy node is accessible...")
                # Note: MoveItPy doesn't expose its node directly, so we can't check parameters
                logger.info("MoveItPy node is internal and not directly accessible")
            except Exception as debug_e:
                logger.warn(f"Could not inspect MoveItPy node: {debug_e}")
            
        except Exception as init_error:
            logger.error("="*60)
            logger.error("DEBUG: MoveItPy Initialization Failed")
            logger.error("="*60)
            logger.error(f"Error type: {type(init_error).__name__}")
            logger.error(f"Error message: {str(init_error)}")
            logger.error(f"Full error: {repr(init_error)}")
            logger.error("Traceback:")
            logger.error(traceback.format_exc())
            
            # Check if it's the QoS override error
            error_str = str(init_error).lower()
            if 'qos' in error_str or 'durability' in error_str or 'clock' in error_str:
                logger.error("="*60)
                logger.error("This appears to be a QoS override error!")
                logger.error("="*60)
                logger.error("Attempting to diagnose...")
                
                # Try without use_sim_time in config
                logger.info("Trying without use_sim_time in config_dict...")
                moveit_config_dict_no_sim = moveit_config_dict.copy()
                if 'use_sim_time' in moveit_config_dict_no_sim:
                    del moveit_config_dict_no_sim['use_sim_time']
                    logger.info("Removed use_sim_time from config_dict")
                
                try:
                    logger.info("Retrying MoveItPy initialization without use_sim_time in config...")
                    xarm_moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict_no_sim)
                    logger.warn("MoveItPy initialized WITHOUT use_sim_time in config")
                    logger.warn("This may cause time synchronization issues!")
                except Exception as retry_error:
                    logger.error(f"Retry also failed: {retry_error}")
                    raise init_error  # Re-raise original error
            
            raise  # Re-raise the original exception
        
        xarm_arm = xarm_moveit.get_planning_component(planning_group)
        logger.info("✓ Planning component obtained")
        
        # Create a robot state to set the joint values
        robot_model = xarm_moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        
        # Set plan start state to current state
        logger.info("Setting start state to current state...")
        xarm_arm.set_start_state_to_current_state()
        logger.info("✓ Start state set")
        
        # Set target joint values (in radians)
        # Adjust these values based on your robot's joint limits
        joint_values = {
            "joint1": 0.0,
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
