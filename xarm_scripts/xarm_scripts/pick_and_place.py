#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanRequestParameters
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
import time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from xarm_scripts.moveit_utils import (
    GROUP, LINK, GRIPPER_CLOSE_POS,
    setup_moveit_config, set_gripper_state, set_gripper,
    pose_to_stamped, move_to_pose
)

# ARM POSITIONS COMPONENTES
APPROACH_Z = 0.3
PICK_EXACT_Z = 0.2
LIFT_Z = 0.4
START_Y = -0.4
GOAL_Y = 0.4
X_LEVEL = -0.3

# OBJETO AGARRADO
# El cubo agarrado va en el TCP, no en el origen de link6. La cadena del URDF es
#     link6 -> link_eef -> xarm_gripper_base_link -> link_tcp   (z + 0.172)
# asi que el centro de la caja adjunta tiene que estar a 0.172 m sobre link6.
# Con z = 0.05 la caja quedaba DENTRO de la muneca: como link6 y todos los links
# del gripper estan en touch_links, esas colisiones se ignoran, y el volumen
# nunca ocupaba el espacio debajo de los dedos donde de verdad esta el cubo. El
# resultado es que el planner no veia el cubo y lo pasaba por dentro de los
# obstaculos.
TCP_OFFSET_Z = 0.172
PICKED_OBJECT_SIZE = [0.05, 0.05, 0.1]


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
    primitive.dimensions = PICKED_OBJECT_SIZE
    box_pose = Pose()
    box_pose.orientation.w = 1.0
    box_pose.position.z = TCP_OFFSET_Z  # centrado en el TCP, no en link6
    picked_object.primitives.append(primitive)
    picked_object.primitive_poses.append(box_pose)
    picked_object.operation = CollisionObject.ADD
    
    attached_object = AttachedCollisionObject()
    attached_object.link_name = LINK
    attached_object.object = picked_object
    attached_object.touch_links = ["xarm_gripper_base_link", "left_finger", "right_finger", "left_inner_knuckle", "right_inner_knuckle", "left_outer_knuckle", "right_outer_knuckle", "link6"]
    
    with planning_scene_monitor.read_write() as scene:
        scene.process_attached_collision_object(attached_object)
        # Sin esto los transforms del cuerpo adjunto quedan sin recalcular y el
        # chequeo de colisiones puede usar una pose vieja.
        scene.current_state.update()
    
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
