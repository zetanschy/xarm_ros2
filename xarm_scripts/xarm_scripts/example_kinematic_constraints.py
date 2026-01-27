#!/usr/bin/env python3
"""
Example demonstrating Kinematic Constraints in MoveIt 2 Python API.

This example shows how to:
- Create joint constraints
- Create position constraints
- Create orientation constraints
- Use constraints in planning

Reference: https://moveit.ai/moveit/ros/python/google/2023/02/15/MoveIt-Humble-Release.html
"""

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
import traceback


def main():
    rclpy.init()
    node = Node("example_kinematic_constraints")
    logger = node.get_logger()
    
    try:
        # Get use_sim_time parameter
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', False)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        logger.info("="*60)
        logger.info("MoveIt 2 Python API: Kinematic Constraints Example")
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
        
        # Example 1: Joint Constraint
        logger.info("\n--- Example 1: Joint Constraint ---")
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
        logger.info("Created joint constraint with target joint values:")
        for joint_name, value in joint_values.items():
            logger.info(f"  {joint_name}: {value:.4f} rad")
        
        # Example 2: Position Constraint (manual creation)
        logger.info("\n--- Example 2: Position Constraint ---")
        end_effector_link = "link6"
        from shape_msgs.msg import SolidPrimitive
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "link_base"
        position_constraint.link_name = end_effector_link
        
        # Create a constraint region (sphere around target position)
        position_constraint.constraint_region.primitives.append(SolidPrimitive())
        position_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        position_constraint.constraint_region.primitives[0].dimensions = [0.01]  # radius
        
        from geometry_msgs.msg import Pose
        target_pose = Pose()
        target_pose.position.x = 0.4
        target_pose.position.y = 0.2
        target_pose.position.z = 0.1
        target_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitive_poses.append(target_pose)
        position_constraint.weight = 1.0
        
        logger.info(f"Created position constraint for {end_effector_link}")
        logger.info(f"  Target position: [0.4, 0.2, 0.1]")
        logger.info(f"  Tolerance (sphere radius): 0.01 m")
        
        # Example 3: Orientation Constraint (manual creation)
        logger.info("\n--- Example 3: Orientation Constraint ---")
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "link_base"
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        logger.info(f"Created orientation constraint for {end_effector_link}")
        logger.info(f"  Target orientation (quaternion): [0.0, 0.0, 0.0, 1.0]")
        logger.info(f"  Tolerance: [0.1, 0.1, 0.1] rad")
        
        # Example 4: Using constraints in planning
        logger.info("\n--- Example 4: Using Constraints in Planning ---")
        logger.info("Setting goal with position constraint...")
        
        # Wrap position constraint in Constraints message
        planning_component.set_goal_state(motion_plan_constraints=[joint_constraint])
        
        logger.info("Planning with position constraint...")
        plan_result = planning_component.plan()
        
        if plan_result:
            logger.info("Planning succeeded!")
            # Execute the plan
            logger.info("Executing trajectory...")
            try:
                xarm_moveit.execute(planning_group, plan_result.trajectory, blocking=True)
                logger.info("✓ Trajectory executed successfully")
            except Exception as e:
                logger.warn(f"Execution failed (this may happen with fake controllers): {e}")
                logger.warn("Continuing with next example...")
        else:
            logger.warn("Planning failed - this is expected if the constraint is infeasible")
        
        import time
        # Wait longer to ensure action client reconnects if needed
        logger.info("Waiting before next example...")
        time.sleep(5)
        
        # Example 5: Combining constraints
        logger.info("\n--- Example 5: Combining Multiple Constraints ---")
        logger.info("You can combine multiple constraints:")
        logger.info("  - Joint constraints: specify exact joint values")
        logger.info("  - Position constraints: specify end-effector position")
        logger.info("  - Orientation constraints: specify end-effector orientation")
        
        # Reset start state to current state before planning next trajectory
        planning_component.set_start_state_to_current_state()
        
        # Create a Constraints message to combine multiple constraints
        combined_constraints_msg = Constraints()
        combined_constraints_msg.position_constraints.append(position_constraint)
        combined_constraints_msg.orientation_constraints.append(orientation_constraint)
        
        logger.info(f"Example: Combining {len(combined_constraints_msg.position_constraints)} position and {len(combined_constraints_msg.orientation_constraints)} orientation constraints")
        logger.info("  (Position + Orientation = Full Pose Constraint)")
        
        # Wrap position constraint in Constraints message
        planning_component.set_goal_state(motion_plan_constraints=[combined_constraints_msg])
        
        logger.info("Planning with combined constraints...")
        plan_result = planning_component.plan()
        
        if plan_result:
            logger.info("Planning succeeded!")
            # Execute the plan
            logger.info("Executing trajectory...")
            try:
                xarm_moveit.execute(planning_group, plan_result.trajectory, blocking=True)
                logger.info("✓ Trajectory executed successfully")
            except Exception as e:
                logger.warn(f"Execution failed (this may happen with fake controllers): {e}")
                logger.warn("This is expected when using fake_controllers - the trajectory was planned successfully.")
        else:
            logger.warn("Planning failed - this is expected if the constraint is infeasible")


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

