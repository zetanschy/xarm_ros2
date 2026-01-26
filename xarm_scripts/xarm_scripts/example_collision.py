#!/usr/bin/env python3
"""
Simple Planning Scene Example for xArm using MoveIt 2 Python API.

This example demonstrates:
1. Setting a target pose for the robot
2. Creating a collision object (box) in the planning scene
3. Adding the collision object to the scene
4. Planning and executing a trajectory that avoids the obstacle

Reference: https://moveit.ai/moveit/ros/python/google/2023/02/15/MoveIt-Humble-Release.html
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
import traceback
import time


def main():
    rclpy.init()
    node = Node("example_pick_place")
    logger = node.get_logger()
    
    try:
        # Get use_sim_time parameter
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', False)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        logger.info("="*60)
        logger.info("MoveIt 2 Python API: Simple Planning Scene Example")
        logger.info("="*60)
        logger.info(f"use_sim_time: {use_sim_time}")
        
        # Configuration
        planning_group = "xarm6"
        end_effector_link = "link6"
        logger.info(f"Initializing MoveItPy for planning group: {planning_group}")
        
        # Use the custom MoveItConfigsBuilder from uf_ros_lib
        moveit_config_builder = MoveItConfigsBuilder(
            context=None,
            controllers_name='fake_controllers',
            dof=6,  # xarm6 has 6 DOF
            robot_type='xarm',
            prefix='',
            limited=True,
        )
        moveit_config_builder.moveit_cpp(
            file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_planning_python.yaml"
        )
        moveit_config_dict = moveit_config_builder.to_dict()
        
        # Add use_sim_time to config dict
        if 'use_sim_time' not in moveit_config_dict:
            moveit_config_dict['use_sim_time'] = use_sim_time
        
        # Apply QoS override fix
        MoveItConfigsBuilder._add_qos_overrides_for_sim_time(moveit_config_dict)
        
        # Initialize MoveItPy
        xarm_moveit = MoveItPy(node_name="moveit_py_pick_place", config_dict=moveit_config_dict)
        xarm_arm = xarm_moveit.get_planning_component(planning_group)
        planning_scene_monitor = xarm_moveit.get_planning_scene_monitor()
        logger.info("✓ MoveItPy initialized successfully")
        
        # Get the planning frame from the planning scene
        with planning_scene_monitor.read_only() as scene:
            planning_frame = scene.planning_frame
            logger.info(f"Planning frame: {planning_frame}")
        
        # ====================================================================
        # Step 1: Set a target pose
        # ====================================================================
        logger.info("\n" + "="*60)
        logger.info("Step 1: Setting Target Pose")
        logger.info("="*60)
        
        # Set plan start state to current state
        xarm_arm.set_start_state_to_current_state()
        
        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = planning_frame
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = 0.4  # Changed from original
        target_pose.pose.position.z = 0.1
        
        logger.info(f"Target pose: position=({target_pose.pose.position.x}, {target_pose.pose.position.y}, {target_pose.pose.position.z})")
        
        # Set the goal state
        xarm_arm.set_goal_state(pose_stamped_msg=target_pose, pose_link=end_effector_link)
        
        # ====================================================================
        # Step 2: Create a collision object (box)
        # ====================================================================
        logger.info("\n" + "="*60)
        logger.info("Step 2: Creating Collision Object")
        logger.info("="*60)
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = planning_frame
        collision_object.id = "box1"
        
        # Define the size of the box in meters
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.2, 0.1, 0.2]  # [BOX_X, BOX_Y, BOX_Z]
        
        # Define the pose of the box (relative to the frame_id)
        box_pose = Pose()
        box_pose.orientation.w = 1.0
        box_pose.position.x = 0.2
        box_pose.position.y = 0.2
        box_pose.position.z = 0.15#0.25
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        logger.info(f"Created collision object '{collision_object.id}' at position ({box_pose.position.x}, {box_pose.position.y}, {box_pose.position.z})")
        logger.info(f"Box dimensions: {primitive.dimensions}")
        
        # ====================================================================
        # Step 3: Add the object to the planning scene
        # ====================================================================
        logger.info("\n" + "="*60)
        logger.info("Step 3: Adding Collision Object to Planning Scene")
        logger.info("="*60)
        
        with planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()
        
        logger.info("✓ Collision object added to planning scene")
        time.sleep(0.5)  # Give time for the scene to update
        
        # ====================================================================
        # Step 4: Plan and execute
        # ====================================================================
        logger.info("\n" + "="*60)
        logger.info("Step 4: Planning and Executing Trajectory")
        logger.info("="*60)
        
        # Plan to the target pose
        logger.info("Planning trajectory...")
        plan_result = xarm_arm.plan()
        
        if not plan_result:
            logger.error("✗ Planning failed!")
            return 1
        
        # Execute the plan
        logger.info("Executing trajectory...")
        xarm_moveit.execute(planning_group, plan_result.trajectory, blocking=True)
        logger.info("✓ Trajectory executed successfully")
        
        logger.info("\n" + "="*60)
        logger.info("Example completed successfully!")
        logger.info("="*60)
        
        return 0

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
