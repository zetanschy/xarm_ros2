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
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from moveit.core.kinematic_constraints import construct_joint_constraint

def main():
    rclpy.init()
    logger = get_logger("moveit_py.xarm_goal")
    
    try:
        logger.info("="*60)
        logger.info("MoveIt2 Python API Test for xArm")
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
        xarm_moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
        xarm_arm = xarm_moveit.get_planning_component(planning_group)
        logger.info("✓ MoveItPy initialized")
        
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
        import traceback
        traceback.print_exc()
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    import sys
    sys.exit(main())
