#!/usr/bin/env python3
"""
Example demonstrating Robot State manipulation in MoveIt 2 Python API.

This example shows how to:
- Get the current robot state
- Query joint positions and velocities
- Set joint positions
- Get link transforms
- Check if a state is valid

Reference: https://moveit.ai/moveit/ros/python/google/2023/02/15/MoveIt-Humble-Release.html
"""

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
import numpy as np
import traceback
import time


def main():
    rclpy.init()
    node = Node("example_robot_state")
    logger = node.get_logger()
    
    try:
        # Get use_sim_time parameter
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', False)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        logger.info("="*60)
        logger.info("MoveIt 2 Python API: Robot State Example")
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
        planning_scene_monitor = xarm_moveit.get_planning_scene_monitor()
        logger.info("MoveItPy initialized successfully")
        
        # Wait for clock synchronization and robot state updates
        # This helps ensure the planning scene monitor has received robot state
        if use_sim_time:
            logger.info("Waiting for clock synchronization (2 seconds)...")
            time.sleep(2.0)
        else:
            logger.info("Waiting for robot state updates (1 second)...")
            time.sleep(1.0)
        
        # Get joint model group
        joint_model_group = robot_model.get_joint_model_group(planning_group)
        
        # Get the current robot state from the planning scene
        logger.info("Getting current robot state from planning scene...")
        with planning_scene_monitor.read_only() as scene:
            robot_state = scene.current_state
        
        # Get joint names from the current state
        joint_names = list(robot_state.joint_positions.keys())
        logger.info(f"Planning group joints: {joint_names}")
        
        # Query joint positions
        logger.info("\n--- Querying Joint Positions ---")
        for joint_name in joint_names:
            joint_value = robot_state.joint_positions[joint_name]
            logger.info(f"  {joint_name}: {joint_value:.4f} rad")
        
        # Get all joint positions as a dictionary
        joint_positions = robot_state.joint_positions
        logger.info(f"\nAll joint positions: {joint_positions}")
        
        # Get link transforms
        logger.info("\n--- Getting Link Transforms ---")
        end_effector_link = "link6"
        transform = robot_state.get_global_link_transform(end_effector_link)
        
        # Transform is a 4x4 numpy array (homogeneous transformation matrix)
        # Translation is in the last column (first 3 elements)
        translation = transform[:3, 3]
        # Rotation matrix is in the upper-left 3x3
        rotation_matrix = transform[:3, :3]
        
        # Convert rotation matrix to quaternion
        # Method: https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
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
            q = [0.0, 0.0, 0.0, 0.0]
            q[i] = 0.25 * s
            q[3] = (rotation_matrix[k, j] - rotation_matrix[j, k]) / s
            q[j] = (rotation_matrix[j, i] + rotation_matrix[i, j]) / s
            q[k] = (rotation_matrix[k, i] + rotation_matrix[i, k]) / s
            w, x, y, z = q[3], q[0], q[1], q[2]
        
        logger.info(f"End effector ({end_effector_link}) transform:")
        logger.info(f"  Translation: [{translation[0]:.4f}, {translation[1]:.4f}, {translation[2]:.4f}]")
        logger.info(f"  Rotation (quaternion): [{w:.4f}, {x:.4f}, {y:.4f}, {z:.4f}]")
        
        # Check if state is valid
        logger.info("\n--- Checking State Validity ---")
        # Use JointModelGroup to check if positions satisfy bounds
        joint_group_positions = robot_state.get_joint_group_positions(planning_group)
        is_valid = joint_model_group.satisfies_position_bounds(joint_group_positions, margin=0.0)
        logger.info(f"State satisfies joint bounds: {is_valid}")
        
        # Get joint velocities (if available)
        logger.info("\n--- Querying Joint Velocities ---")
        for joint_name in joint_names:
            try:
                # Joint velocities may not always be available
                if hasattr(robot_state, 'joint_velocities') and robot_state.joint_velocities:
                    joint_velocity = robot_state.joint_velocities.get(joint_name, 0.0)
                    logger.info(f"  {joint_name}: {joint_velocity:.4f} rad/s")
                else:
                    logger.info(f"  {joint_name}: velocity not available")
            except Exception as e:
                logger.debug(f"  {joint_name}: velocity not available ({e})")
        
        logger.info("\n" + "="*60)
        logger.info("Robot State example completed successfully!")
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

