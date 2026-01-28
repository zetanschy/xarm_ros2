#!/usr/bin/env python3
"""
Pick and Place Script for xArm Robot

This script performs a pick and place operation:
1. Moves to approach position above the beer
2. Moves down to pick position
3. Grasps the beer (closes gripper)
4. Moves up to lift position
5. Moves to place position above target
6. Moves down to place position
7. Releases the beer (opens gripper)
8. Moves up to retract position

Usage:
    ros2 run xarm_scripts pick_and_place_beer
"""

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
import time
import traceback


def set_gripper_named_state(xarm_moveit, state_name="open", logger=None):
    """Set gripper to a named state from SRDF (e.g., 'open', 'close') using MoveIt
    
    This follows the exact same pattern as example_joint_goal.py for the 'home' state.
    Group name is 'xarm_gripper' as confirmed in RViz.
    """
    gripper_group = "xarm_gripper"
    
    try:
        # Get planning component for gripper (same as xarm_arm = xarm_moveit.get_planning_component("xarm6"))
        gripper_component = xarm_moveit.get_planning_component(gripper_group)
        
        if logger:
            logger.info(f"Setting gripper to '{state_name}' state (group: {gripper_group})")
        
        # Set start state to current state (exactly like example_joint_goal.py line 119)
        gripper_component.set_start_state_to_current_state()
        
        # Check available named target states (exactly like example_joint_goal.py line 122)
        named_targets = gripper_component.named_target_states
        if logger:
            logger.info(f"Available named target states: {named_targets}")
        
        # Set goal state using named configuration from SRDF (exactly like example_joint_goal.py line 127)
        if logger:
            logger.info(f"Using '{state_name}' configuration from SRDF")
        gripper_component.set_goal_state(configuration_name=state_name)
        
        # Plan (exactly like example_joint_goal.py line 131)
        if logger:
            logger.info(f"Planning to {state_name} position...")
        plan_result = gripper_component.plan()
        
        if not plan_result:
            if logger:
                logger.warn(f"Planning to {state_name} position failed!")
            return False
        
        # Execute (exactly like example_joint_goal.py line 138)
        gripper_trajectory = plan_result.trajectory
        if logger:
            logger.info(f"Executing trajectory to {state_name} position...")
        xarm_moveit.execute(gripper_group, gripper_trajectory, blocking=True)
        
        if logger:
            logger.info(f"✓ Gripper moved to '{state_name}' position")
        return True
        
    except Exception as e:
        if logger:
            logger.error(f"Error setting gripper to '{state_name}' state: {e}")
        import traceback
        traceback.print_exc()
        return False


def set_gripper_position_trajectory(node, gripper_traj_client, position_rad, wait=True, timeout=10.0, current_position=None):
    """Set gripper position using FollowJointTrajectory action (0.0=fully open, 0.86=fully closed)"""
    try:
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory - match MoveIt format
        trajectory = JointTrajectory()
        trajectory.joint_names = ['drive_joint']
        trajectory.header.frame_id = ''  # Empty frame_id like MoveIt uses
        trajectory.header.stamp = node.get_clock().now().to_msg()
        
        # Create target trajectory point (controller knows current state)
        point = JointTrajectoryPoint()
        point.positions = [float(position_rad)]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        # Set trajectory duration to 2 seconds
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        # Add path tolerances (like MoveIt does)
        path_tolerance = JointTolerance()
        path_tolerance.name = 'drive_joint'
        path_tolerance.position = 0.1  # Allow some tolerance
        path_tolerance.velocity = 0.1
        path_tolerance.acceleration = 0.1
        goal_msg.path_tolerance = [path_tolerance]
        
        # Add goal tolerances
        goal_tolerance = JointTolerance()
        goal_tolerance.name = 'drive_joint'
        goal_tolerance.position = 0.05  # Tighter goal tolerance
        goal_tolerance.velocity = 0.1
        goal_tolerance.acceleration = 0.1
        goal_msg.goal_tolerance = [goal_tolerance]
        
        # Goal time tolerance (allow some time flexibility)
        goal_msg.goal_time_tolerance.sec = 1
        goal_msg.goal_time_tolerance.nanosec = 0
        
        if not gripper_traj_client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error("Gripper trajectory action server not available")
            return False
        
        node.get_logger().info(f"Sending gripper trajectory goal: position={position_rad:.3f} rad")
        send_goal_future = gripper_traj_client.send_goal_async(goal_msg)
        
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done():
            node.get_logger().error("Failed to send gripper goal")
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            node.get_logger().warn("Gripper goal rejected, retrying with updated timestamp...")
            # Retry with fresh timestamp
            goal_msg.trajectory.header.stamp = node.get_clock().now().to_msg()
            node.get_logger().info(f"Retrying gripper trajectory goal: position={position_rad:.3f} rad")
            send_goal_future = gripper_traj_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
            
            if not send_goal_future.done():
                node.get_logger().error("Failed to send retry gripper goal")
                return False
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                node.get_logger().error("Gripper goal rejected on retry as well")
                return False
            else:
                node.get_logger().info("✓ Gripper goal accepted on retry")
        
        if wait:
            node.get_logger().info("Waiting for gripper trajectory to complete...")
            get_result_future = goal_handle.get_result_async()
            
            # Wait for result with timeout - use spin_until_future_complete
            rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=timeout)
            
            if get_result_future.done():
                try:
                    result = get_result_future.result().result
                    if result.error_code == 0:
                        node.get_logger().info(f"✓ Gripper moved to position {position_rad:.3f} rad")
                        return True
                    else:
                        error_string = result.error_string if hasattr(result, 'error_string') and result.error_string else "Unknown error"
                        node.get_logger().warn(f"Gripper trajectory completed with error code: {result.error_code}, error: {error_string}")
                        return False
                except Exception as e:
                    node.get_logger().error(f"Error getting result: {e}")
                    return False
            else:
                # Check goal status
                status = goal_handle.status
                node.get_logger().warn(f"Gripper trajectory timed out after {timeout} seconds. Goal status: {status}")
                # Try to cancel the goal
                try:
                    goal_handle.cancel_goal_async()
                except:
                    pass
                return False
        else:
            node.get_logger().info("Gripper goal sent (not waiting for completion)")
            return True
            
    except Exception as e:
        node.get_logger().error(f"Error setting gripper position: {e}")
        import traceback
        traceback.print_exc()
        return False


def move_to_pose(planning_component, xarm_moveit, planning_group, 
                 position, end_effector_link="link6", orientation=None, description="", logger=None,
                 velocity_scale=1.0):
    """Move end effector to a specific pose
    
    Args:
        velocity_scale: Scale factor for velocity (0.0-1.0). Lower = slower movement.
    """
    if description and logger:
        logger.info(f"Moving to {description}...")
    
    # Set start state to current state
    planning_component.set_start_state_to_current_state()
    
    # Create pose goal
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"
    pose_goal.pose.position.x = float(position[0])
    pose_goal.pose.position.y = float(position[1])
    pose_goal.pose.position.z = float(position[2])
    
    # Set orientation (default: pointing down)
    if orientation is None:
        # Default orientation: gripper pointing down (common for pick and place)
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 1.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.0
    else:
        pose_goal.pose.orientation.x = float(orientation[0])
        pose_goal.pose.orientation.y = float(orientation[1])
        pose_goal.pose.orientation.z = float(orientation[2])
        pose_goal.pose.orientation.w = float(orientation[3])
    
    # Set goal
    planning_component.set_goal_state(
        pose_stamped_msg=pose_goal,
        pose_link=end_effector_link
    )
    
    # Plan
    plan_result = planning_component.plan()
    
    if not plan_result:
        if logger:
            logger.error(f"Planning failed for {description}")
        return False
    
    # Apply velocity scaling if needed (slow down movements)
    if velocity_scale < 1.0:
        trajectory = plan_result.trajectory
        # Scale time_from_start for each point to slow down execution
        try:
            robot_traj_msg = trajectory.get_robot_trajectory_msg()
            for point in robot_traj_msg.joint_trajectory.points:
                # Scale velocities
                point.velocities = [v * velocity_scale for v in point.velocities]
                # Scale accelerations  
                point.accelerations = [a * velocity_scale * velocity_scale for a in point.accelerations]
                # Extend time proportionally
                original_secs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                new_secs = original_secs / velocity_scale
                point.time_from_start.sec = int(new_secs)
                point.time_from_start.nanosec = int((new_secs - int(new_secs)) * 1e9)
        except Exception as e:
            if logger:
                logger.warn(f"Could not apply velocity scaling: {e}")
    
    # Execute
    try:
        xarm_moveit.execute(
            planning_group,
            plan_result.trajectory,
            blocking=True
        )
        if description and logger:
            logger.info(f"✓ Reached {description}")
        return True
    except Exception as e:
        if logger:
            logger.error(f"Execution failed: {e}")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Node("pick_and_place_beer")
    logger = node.get_logger()
    
    try:
        # Get use_sim_time parameter (default to True for simulation)
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', True)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        logger.info("="*60)
        logger.info("Pick and Place Script for xArm Robot")
        logger.info("="*60)
        logger.info(f"use_sim_time: {use_sim_time}")
        
        # Configuration
        planning_group = "xarm6"  # Change to "xarm7", "xarm5", etc. as needed
        end_effector_link = "link6"
        logger.info(f"Initializing MoveItPy for planning group: {planning_group}")
        
        # Build MoveIt configuration
        moveit_config_builder = MoveItConfigsBuilder(
            context=None,
            controllers_name='fake_controllers',
            dof=6,  # xarm6 has 6 DOF
            robot_type='xarm',
            prefix='',
            limited=True,
            add_gripper=True,  # Include gripper group (xarm_gripper) in SRDF
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
        planning_component = xarm_moveit.get_planning_component(planning_group)
        logger.info("✓ MoveItPy initialized successfully")
        
        # Initialize gripper action client (preferred method for MoveIt/ros2_control)
        logger.info("Initializing gripper trajectory action client...")
        gripper_traj_client = ActionClient(node, FollowJointTrajectory, '/xarm_gripper_traj_controller/follow_joint_trajectory')
        
        # Wait for gripper action server
        logger.info("Waiting for gripper trajectory action server...")
        if not gripper_traj_client.wait_for_server(timeout_sec=5.0):
            logger.error("✗ Gripper trajectory action server not available!")
            logger.error("  Make sure you launch with: add_gripper:=true")
            return 1
        
        logger.info("✓ Gripper trajectory action server is available")
        
        # Pick and place positions (in link_base frame)
        # Adjust these based on your table setup and beer position
        pick_approach = [-0.3, -0.4, 0.3]  # Above the beer
        pick_position = [-0.3, -0.4, 0.2]  # At the beer (lower z)
        lift_position = [-0.3, -0.4, 0.4]  # Lift height after grasping
        
        # Place position (another location on the table)
        place_approach = [-0.3, 0.4, 0.4]   # Above the place location
        place_position = [-0.3, 0.4, 0.2]   # At the place location
        retract_position = [-0.3, 0.4, 0.3] # Retract height after placing
        
        # Gripper settings - position in radians (0.0 = fully open, 0.86 = fully closed)
        gripper_open_pos_rad = 0.0    # Fully open
        gripper_close_pos_rad = 0.5  # Close tightly (adjust based on box size)
        
        # Wait a moment for everything to initialize
        time.sleep(2.0)
        
        # Execute pick and place sequence
        logger.info("="*60)
        logger.info("Starting Pick and Place Sequence")
        logger.info("="*60)
        
        # Step 3: Move to approach position above beer
        if not move_to_pose(planning_component, xarm_moveit, planning_group,
                           pick_approach, end_effector_link, 
                           description="pick approach position", logger=logger):
            return 1
        time.sleep(0.5)
        
        # Step 4: Move down to pick position
        if not move_to_pose(planning_component, xarm_moveit, planning_group,
                           pick_position, end_effector_link,
                           description="pick position", logger=logger):
            return 1
        time.sleep(0.5)
        
        # Step 5: Close gripper (grasp)
        logger.info("Closing gripper (grasping)...")
        set_gripper_position_trajectory(node, gripper_traj_client, gripper_close_pos_rad, wait=True, timeout=10.0)
        time.sleep(2.5)  # Wait for gripper to close and physics to settle
        
        # Step 6: Lift up (slowly to prevent object from slipping)
        if not move_to_pose(planning_component, xarm_moveit, planning_group,
                           lift_position, end_effector_link,
                           description="lift position", logger=logger,
                           velocity_scale=0.3):  # Slow movement while carrying
            return 1
        time.sleep(0.5)
        
        # Step 7: Move to approach position above place location
        if not move_to_pose(planning_component, xarm_moveit, planning_group,
                           place_approach, end_effector_link,
                           description="place approach position", logger=logger,
                           velocity_scale=0.3):  # Slow movement while carrying
            return 1
        time.sleep(0.5)
        
        # Step 8: Move down to place position
        if not move_to_pose(planning_component, xarm_moveit, planning_group,
                           place_position, end_effector_link,
                           description="place position", logger=logger,
                           velocity_scale=0.3):  # Slow movement while carrying
            return 1
        time.sleep(0.5)
        
        # Step 9: Open gripper (release/free) - using MoveIt named state
        logger.info("Opening gripper (releasing)...")
        set_gripper_named_state(xarm_moveit, state_name="open", logger=logger)
        time.sleep(1.5)  # Wait for gripper to open
        
        # Step 10: Retract up
        if not move_to_pose(planning_component, xarm_moveit, planning_group,
                           retract_position, end_effector_link,
                           description="retract position", logger=logger):
            return 1
        
        logger.info("="*60)
        logger.info("✓ Pick and Place Sequence Completed Successfully!")
        logger.info("="*60)
        
    except Exception as e:
        logger.error(f"✗ Error: {str(e)}")
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main(args=sys.argv))
