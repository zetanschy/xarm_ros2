#!/usr/bin/env python3
"""
Example demonstrating Multi-Pipeline Planning in MoveIt 2 Python API.

This example shows how to:
- Use different planning pipelines (OMPL, Pilz, CHOMP, STOMP)
- Try multiple pipelines to find a solution
- Compare results from different planners
- Execute plans from different pipelines

Reference: https://moveit.ai/moveit/ros/python/google/2023/02/15/MoveIt-Humble-Release.html
"""

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import time
import traceback


def plan_and_execute(moveit_py, planning_component, logger, plan_parameters=None, multi_plan_parameters=None):
    """
    Plan and execute a motion.
    
    Args:
        moveit_py: MoveItPy instance
        planning_component: PlanningComponent instance
        logger: Logger instance
        plan_parameters: Optional PlanRequestParameters for single pipeline
        multi_plan_parameters: Optional list of PlanRequestParameters for multi-pipeline
    """
    plan_result = None
    
    if multi_plan_parameters:
        # Try multiple pipelines
        logger.info(f"Trying {len(multi_plan_parameters)} planning pipelines...")
        for i, params in enumerate(multi_plan_parameters):
            logger.info(f"Pipeline {i+1}/{len(multi_plan_parameters)}: {params.planning_pipeline}")
            plan_result = planning_component.plan(params)
            if plan_result:
                logger.info(f"✓ Planning succeeded with pipeline: {params.planning_pipeline}")
                break
            else:
                logger.warn(f"✗ Planning failed with pipeline: {params.planning_pipeline}")
    else:
        # Single pipeline planning
        logger.info("Planning...")
        plan_result = planning_component.plan(plan_parameters)
    
    if plan_result:
        logger.info("Planning succeeded!")
        logger.info(f"Trajectory has {len(plan_result.trajectory)} waypoints")
        
        # Execute the plan
        logger.info("Executing plan...")
        moveit_py.execute(
            planning_component.planning_group_name,
            plan_result.trajectory,
            blocking=True
        )
        logger.info("Execution completed!")
        return plan_result
    else:
        logger.warn("Planning failed with all pipelines!")
        return None


def main():
    rclpy.init()
    node = Node("example_multi_pipeline")
    logger = node.get_logger()
    
    try:
        # Get use_sim_time parameter
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', False)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        
        logger.info("="*60)
        logger.info("MoveIt 2 Python API: Multi-Pipeline Planning Example")
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
        xarm_arm = xarm_moveit.get_planning_component(planning_group)
        logger.info("MoveItPy initialized successfully")
        
        # Set plan start state to current state
        logger.info("\n--- Setting Start State ---")
        xarm_arm.set_start_state_to_current_state()
        logger.info("Start state set to current state")
        
        # Example 1: Plan with default pipeline
        logger.info("\n--- Example 1: Planning with Default Pipeline ---")
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "link_base"
        pose_goal.pose.position.x = 0.4
        pose_goal.pose.position.y = 0.2
        pose_goal.pose.position.z = 0.2
        pose_goal.pose.orientation.w = 1.0
        
        xarm_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        logger.info(f"Goal pose: [{pose_goal.pose.position.x}, {pose_goal.pose.position.y}, {pose_goal.pose.position.z}]")
        
        plan_result = xarm_arm.plan()
        if plan_result:
            logger.info(f"Default pipeline planning succeeded! Trajectory has {len(plan_result.trajectory)} waypoints")
        else:
            logger.warn("Default pipeline planning failed")
        
        # Example 2: Multi-pipeline planning with configuration name (tutorial pattern)
        logger.info("\n--- Example 2: Multi-Pipeline Planning with Configuration Name ---")
        
        # Set plan start state to current state
        xarm_arm.set_start_state_to_current_state()
        
        # Set goal state using configuration name (if available)
        named_targets = xarm_arm.named_target_states
        if named_targets:
            target_name = named_targets[0]
            logger.info(f"Setting goal to named configuration: '{target_name}'")
            xarm_arm.set_goal_state(configuration_name=target_name)
        else:
            # Fallback to pose goal if no named targets
            logger.info("No named targets available, using pose goal")
            xarm_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        
        # Create multi-pipeline plan request parameters
        pipeline_configs = ["ompl"]  # Start with OMPL which is most commonly available
        
        multi_pipeline_plan_request_params = []
        for pipeline_name in pipeline_configs:
            params = PlanRequestParameters(xarm_moveit)
            params.planning_pipeline = pipeline_name
            params.planner_id = ""  # Use default planner for the pipeline
            params.planning_time = 5.0
            params.planning_attempts = 1
            multi_pipeline_plan_request_params.append(params)
        
        logger.info(f"Created {len(multi_pipeline_plan_request_params)} pipeline configurations")
        
        # Plan to goal using multi-pipeline approach
        plan_result = plan_and_execute(
            xarm_moveit,
            xarm_arm,
            logger,
            multi_plan_parameters=multi_pipeline_plan_request_params,
        )
        
        # Execute the plan (if plan_and_execute didn't execute it)
        if plan_result:
            logger.info("Executing plan...")
            xarm_moveit.execute(
                planning_group,
                plan_result.trajectory,
                blocking=True
            )
            logger.info("Execution completed!")
        
        # Example 3: Simple multi-pipeline planning with pose goal
        logger.info("\n--- Example 3: Multi-Pipeline Planning with Pose Goal ---")
        
        xarm_arm.set_start_state_to_current_state()
        xarm_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        
        # Create parameters for multiple pipelines
        pipeline_configs = []
        for pipeline_name in ["ompl"]:  # Add more pipelines if available
            params = PlanRequestParameters(xarm_moveit)
            params.planning_pipeline = pipeline_name
            params.planner_id = ""
            params.planning_time = 5.0
            pipeline_configs.append(params)
        
        # Try each pipeline
        for params in pipeline_configs:
            logger.info(f"Trying pipeline: {params.planning_pipeline}")
            plan_result = xarm_arm.plan(params)
            if plan_result:
                logger.info(f"✓ Success with {params.planning_pipeline}")
                break
        
        # Example 4: Compare different planners within OMPL pipeline
        logger.info("\n--- Example 4: Comparing Different OMPL Planners ---")
        
        # Nombres tal como los declara config/xarm6/ompl_planning.yaml.
        # OJO: el sufijo "kConfigDefault" es la convencion de MoveIt 1 y NO existe
        # en este repo; si se usa, OMPL avisa "Cannot find planning configuration"
        # y cae silenciosamente a RRTConnect, con lo que los cuatro planners
        # resultan ser el mismo y la comparacion no compara nada.
        ompl_planners = [
            "RRTConnect",
            "RRTstar",
            "PRM",
            "LBKPIECE",
        ]
        
        xarm_arm.set_start_state_to_current_state()
        xarm_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        
        for planner_id in ompl_planners:
            logger.info(f"\nTrying OMPL planner: {planner_id}")
            
            plan_parameters = PlanRequestParameters(xarm_moveit)
            plan_parameters.planning_pipeline = "ompl"
            plan_parameters.planner_id = planner_id
            plan_parameters.planning_time = 5.0
            plan_parameters.planning_attempts = 1
            
            # Medir el tiempo de PLANIFICACION: trajectory.duration es otra cosa
            # (lo que tarda el robot en ejecutar la trayectoria, no en encontrarla).
            t_start = time.perf_counter()
            plan_result = xarm_arm.plan(plan_parameters)
            planning_time = time.perf_counter() - t_start
            
            if plan_result:
                logger.info(f"✓ Planner '{planner_id}' succeeded!")
                logger.info(f"  Planning time: {planning_time:.4f}s   <-- cuanto tardo en PLANIFICAR")
                logger.info(f"  Waypoints: {len(plan_result.trajectory)}")
                logger.info(f"  Trajectory duration: {plan_result.trajectory.duration:.4f}s   <-- cuanto tarda en EJECUTARSE")
            else:
                logger.warn(f"✗ Planner '{planner_id}' failed after {planning_time:.4f}s")
        
        # Example 5: Using plan_and_execute helper function
        logger.info("\n--- Example 5: Using plan_and_execute Helper ---")
        xarm_arm.set_start_state_to_current_state()
        xarm_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        
        # Create plan parameters
        plan_parameters = PlanRequestParameters(xarm_moveit)
        plan_parameters.planning_pipeline = "ompl"
        plan_parameters.planner_id = "RRTConnect"
        plan_parameters.planning_time = 5.0
        
        plan_and_execute(xarm_moveit, xarm_arm, logger, plan_parameters)

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

