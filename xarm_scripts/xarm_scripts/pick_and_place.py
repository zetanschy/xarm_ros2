#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanRequestParameters
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
import time
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive

GROUP = "xarm6"
LINK = "link6"

# ARM POSITIONS COMPONENTES
APPROACH_Z = 0.3
PICK_EXACT_Z = 0.2
LIFT_Z = 0.4
START_Y = -0.4
GOAL_Y = 0.4
X_LEVEL = -0.3

# GRIPPER POSE
GRIPPER_CLOSE_POS = 0.5

def setup_moveit_config(node, dof=6, add_gripper=True):
    if node.has_parameter('use_sim_time'):
        use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
    else:
        node.declare_parameter('use_sim_time', False)
        use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
    
    node.get_logger().info(f"use_sim_time: {use_sim_time}")
    
    moveit_config_builder = MoveItConfigsBuilder(
        context=None,
        controllers_name='fake_controllers',
        dof=dof,
        robot_type='xarm',
        prefix='',
        limited=True,
        add_gripper=add_gripper,
    )
    moveit_config_builder.moveit_cpp(
        file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_planning_python.yaml"
    )
    moveit_config_dict = moveit_config_builder.to_dict()
    
    moveit_config_dict['use_sim_time'] = use_sim_time
    
    MoveItConfigsBuilder._add_qos_overrides_for_sim_time(moveit_config_dict)
    
    return moveit_config_dict, use_sim_time

def set_gripper_state(xarm_moveit, state_name):
    gripper = xarm_moveit.get_planning_component("xarm_gripper")
    gripper.set_start_state_to_current_state()
    gripper.set_goal_state(configuration_name=state_name)
    plan_result = gripper.plan()
    if plan_result:
        xarm_moveit.execute("xarm_gripper", plan_result.trajectory, blocking=True)

def set_gripper(node, gripper_client, position_rad):
    goal_msg = FollowJointTrajectory.Goal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['drive_joint']
    trajectory.header.stamp = node.get_clock().now().to_msg()
    
    point = JointTrajectoryPoint()
    point.positions = [float(position_rad)]
    point.velocities = [0.0]
    point.accelerations = [0.0]
    point.time_from_start.sec = 2
    trajectory.points = [point]
    goal_msg.trajectory = trajectory
    
    tolerance = JointTolerance()
    tolerance.name = 'drive_joint'
    tolerance.position = 0.1
    goal_msg.path_tolerance = [tolerance]
    goal_msg.goal_tolerance = [tolerance]
    goal_msg.goal_time_tolerance.sec = 1
    
    send_future = gripper_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    goal_handle = send_future.result()
    
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)

def pose_to_stamped(position):
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"
    pose_goal.pose.position.x = float(position[0])
    pose_goal.pose.position.y = float(position[1])
    pose_goal.pose.position.z = float(position[2])
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 1.0
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.0
    return pose_goal

def move_to_pose(planning_component, xarm_moveit, group, position, link="link6"):
    planning_component.set_start_state_to_current_state()
    
    pose_goal = pose_to_stamped(position)
    
    planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=link)
    
    plan_result = planning_component.plan()
    
    if plan_result:
        xarm_moveit.execute(group, plan_result.trajectory, blocking=True)
        return True
    return False

def main(args=None):
    rclpy.init(args=args)
    node = Node("pick_and_place")
    
    config_dict, _ = setup_moveit_config(node, dof=6, add_gripper=True)
    xarm_moveit = MoveItPy(node_name="moveit_py", config_dict=config_dict)
    planning = xarm_moveit.get_planning_component(GROUP)
    
    gripper_client = ActionClient(node, FollowJointTrajectory, '/xarm_gripper_traj_controller/follow_joint_trajectory')
    gripper_client.wait_for_server(timeout_sec=5.0)
    
    pick_approach = [X_LEVEL, START_Y, APPROACH_Z]
    pick_position = [X_LEVEL, START_Y, PICK_EXACT_Z]
    lift_position = [X_LEVEL, START_Y, LIFT_Z]
    place_approach = [X_LEVEL, GOAL_Y, LIFT_Z]
    place_position = [X_LEVEL, GOAL_Y, PICK_EXACT_Z]
    retract_position = [X_LEVEL, GOAL_Y, APPROACH_Z]
    
    time.sleep(2.0)
    
    # 1) PICK ACTION
    move_to_pose(planning, xarm_moveit, GROUP, pick_approach, LINK)
    move_to_pose(planning, xarm_moveit, GROUP, pick_position, LINK)
    set_gripper(node, gripper_client, GRIPPER_CLOSE_POS)
    time.sleep(1.0)
    
    planning_scene_monitor = xarm_moveit.get_planning_scene_monitor()
    
    picked_object = CollisionObject()
    picked_object.header.frame_id = LINK
    picked_object.id = "picked_object"
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.05, 0.05, 0.1]
    box_pose = Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.z = 0.05
    picked_object.primitives.append(primitive)
    picked_object.primitive_poses.append(box_pose)
    picked_object.operation = CollisionObject.ADD
    
    attached_object = AttachedCollisionObject()
    attached_object.link_name = LINK
    attached_object.object = picked_object
    attached_object.touch_links = ["xarm_gripper_base_link", "left_finger", "right_finger", "left_inner_knuckle", "right_inner_knuckle", "left_outer_knuckle", "right_outer_knuckle", "link6"]
    
    with planning_scene_monitor.read_write() as scene:
        scene.process_attached_collision_object(attached_object)
    
    move_to_pose(planning, xarm_moveit, GROUP, lift_position, LINK)

    # 2) COLLISION AVOIDANCE
    # 2.1) ADDING TO SCENE
    collision_object = CollisionObject()
    with planning_scene_monitor.read_only() as scene:
        planning_frame = scene.planning_frame
    collision_object.header.frame_id = planning_frame
    collision_object.id = "obstacle"
    
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.04, 0.5, 0.35]
    
    box_pose = Pose() # Define the pose of the box (relative to the frame_id)
    box_pose.orientation.w = 1.0
    box_pose.position.x = 0.0
    box_pose.position.y = 0.5
    box_pose.position.z = 0.15
    
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD

    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(collision_object)
        scene.current_state.update()
    
    time.sleep(1.0)  # Give time for the scene to update
    
    # 2.2) MULTI-PIPELINE AND COLLISION AVOIDANCE
    planning.set_start_state_to_current_state()
    planning.set_goal_state(pose_stamped_msg=pose_to_stamped(place_approach), pose_link=LINK)
    times = {}
    for planner_id in ["RRTConnectkConfigDefault", "RRTstarkConfigDefault", "PRMkConfigDefault"]:
        plan_parameters = PlanRequestParameters(xarm_moveit)
        plan_parameters.planning_pipeline = "ompl"
        plan_parameters.planner_id = planner_id
        plan_result = planning.plan(plan_parameters)
        if plan_result:
            duration = plan_result.trajectory.duration
            times[planner_id] = duration
            print(f"{planner_id}: {duration:.4f}s")
    
    if times:
        best_planner = min(times, key=times.get)
        plan_parameters = PlanRequestParameters(xarm_moveit)
        plan_parameters.planning_pipeline = "ompl"
        plan_parameters.planner_id = best_planner
        plan_result = planning.plan(plan_parameters)
        if plan_result:
            xarm_moveit.execute(GROUP, plan_result.trajectory, blocking=True)
    
    # 3) PLACE ACTION
    move_to_pose(planning, xarm_moveit, GROUP, place_position, LINK)
    set_gripper_state(xarm_moveit, "open")
    time.sleep(1.0)
    
    picked_object.operation = CollisionObject.REMOVE
    attached_object.object = picked_object
    with planning_scene_monitor.read_write() as scene:
        scene.process_attached_collision_object(attached_object)
        scene.apply_collision_object(picked_object)
    time.sleep(0.5)
    
    move_to_pose(planning, xarm_moveit, GROUP, retract_position, LINK)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
