#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt Task Constructor (MTC) Pick and Place Demo for xArm6 - EXECUTION VERSION

This script plans AND executes the pick-and-place task in simulation.
It waits for move_group to be ready before publishing the collision object.

Now accepts ROS2 params:
  - pick_x (double)
  - pick_y (double)
"""

import rclpy
from rclpy.node import Node as RclpyNode

import rclcpp
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
import math
import time


def publish_collision_object(node: RclpyNode, object_name, pose_stamped, dimensions):
    """
    Wait for move_group, then publish a collision object to the planning scene.
    Verifies the object actually appears before returning.
    """
    # ── Wait for move_group to be ready ──
    cli = node.create_client(GetPlanningScene, '/get_planning_scene')
    print("[INFO] Waiting for move_group (/get_planning_scene service)...")
    while not cli.wait_for_service(timeout_sec=2.0):
        print("[INFO]   ... still waiting for move_group ...")
    print("[INFO] move_group is ready!")

    # Give move_group a moment to fully initialise its planning scene monitor
    time.sleep(3.0)

    # ── Build the collision object message ──
    co = CollisionObject()
    co.header.frame_id = pose_stamped.header.frame_id
    co.id = object_name
    co.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(dimensions)
    co.primitives.append(box)
    co.primitive_poses.append(pose_stamped.pose)

    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects.append(co)

    # ── Publish multiple times for reliability ──
    pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
    time.sleep(1.0)  # Wait for publisher discovery
    for _ in range(5):
        pub.publish(ps)
        time.sleep(0.3)

    print(
        f"[INFO] Published collision object '{object_name}' at "
        f"({pose_stamped.pose.position.x}, {pose_stamped.pose.position.y}, "
        f"{pose_stamped.pose.position.z}) in '{pose_stamped.header.frame_id}'"
    )

    # ── Verify the object is in the planning scene ──
    req = GetPlanningScene.Request()
    req.components.components = req.components.WORLD_OBJECT_NAMES
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.result() is not None:
        names = [o.id for o in future.result().scene.world.collision_objects]
        if object_name in names:
            print(f"[INFO] Verified: '{object_name}' is in the planning scene.")
        else:
            print(f"[WARN] '{object_name}' NOT found in scene (objects: {names}). Retrying...")
            for _ in range(5):
                pub.publish(ps)
                time.sleep(0.5)
            time.sleep(1.0)
    else:
        print("[WARN] Could not verify planning scene (service call timed out).")

    time.sleep(1.0)


def main():
    # ── Robot parameters ──
    arm = "xarm6"
    eef = "xarm_gripper"

    # ── Object parameters ──
    object_name = "object"
    object_size = [0.05, 0.05, 0.1]

    # ============================================================
    # 1) Read pick_x / pick_y from ROS2 params (from your launch)
    # ============================================================
    rclpy.init()

    # IMPORTANT:
    # Your launch passes a *lot* of MoveIt params (moveit_config_dict).
    # If we don't auto-declare overrides, rclpy node creation can fail.
    param_node = RclpyNode(
        "mtc_pickplace_execute",
        automatically_declare_parameters_from_overrides=True,
    )

    def get_param(name: str, default):
        if not param_node.has_parameter(name):
            param_node.declare_parameter(name, default)
        return param_node.get_parameter(name).value

    x_start = float(get_param("pick_x", -0.3))
    y_start = float(get_param("pick_y", -0.4))

    print(f"[INFO] Using pick location from params: pick_x={x_start:.4f}, pick_y={y_start:.4f}")

    # ── Place (trash can) fixed pose ──
    CAN_X = -0.2
    CAN_Y = -0.58
    CAN_Z = 0.3
    x_end, y_end, z_end = CAN_X, CAN_Y, CAN_Z

    MIN = 0.03
    MAX = 0.07
    SPEED = 0.5

    # Object pose at the pick location
    objectPose = PoseStamped()
    objectPose.header.frame_id = "link_base"
    objectPose.pose.orientation.w = 1.0
    objectPose.pose.position.x = x_start
    objectPose.pose.position.y = y_start
    objectPose.pose.position.z = -0.025

    # ── 2. Wait for move_group, then add collision object ──
    print("[INFO] Setting up collision object...")
    publish_collision_object(param_node, object_name, objectPose, object_size)

    param_node.destroy_node()
    rclpy.shutdown()

    # ============================================================
    # 3) MTC task (via rclcpp) - unchanged except it uses scene object
    # ============================================================
    rclcpp.init()

    node_options = rclcpp.NodeOptions()
    node_options.automatically_declare_parameters_from_overrides = True
    node = rclcpp.Node("mtc_pickplace_execute", node_options)

    task = core.Task()
    task.name = "pick + place"
    task.loadRobotModel(node)

    task.add(stages.CurrentState("current"))

    pipeline = core.PipelinePlanner(node)
    pipeline.planner = "RRTConnect"
    planners = [(arm, pipeline)]

    task.add(stages.Connect("connect", planners))

    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
    grasp_generator.angle_delta = math.pi / 2
    grasp_generator.pregrasp = "open"
    grasp_generator.grasp = "grasp"
    grasp_generator.setMonitoredStage(task["current"])

    simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")

    ik_frame = PoseStamped()
    ik_frame.header.frame_id = "link_tcp"
    ik_frame.pose.position.z = 0.0
    ik_frame.pose.orientation.x = 1.0
    ik_frame.pose.orientation.w = 0.0
    simpleGrasp.setIKFrame(ik_frame)

    pick = stages.Pick(simpleGrasp, "Pick")
    pick.eef = eef
    pick.object = object_name

    approach = TwistStamped()
    approach.header.frame_id = "link_base"
    approach.twist.linear.z = -SPEED
    pick.setApproachMotion(approach, MIN, MAX)

    lift = TwistStamped()
    lift.header.frame_id = "link_tcp"
    lift.twist.linear.z = -SPEED
    pick.setLiftMotion(lift, MIN, MAX)

    task.add(pick)

    con = stages.Connect("connect2", planners)
    task.add(con)

    placePose = PoseStamped()
    placePose.header.frame_id = "link_base"
    placePose.pose.orientation.w = 1.0
    placePose.pose.position.x = x_end
    placePose.pose.position.y = y_end
    placePose.pose.position.z = z_end

    place_generator = stages.GeneratePlacePose("Generate Place Pose")
    place_generator.setMonitoredStage(task["Pick"])
    place_generator.object = object_name
    place_generator.pose = placePose

    simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp")

    place = stages.Place(simpleUnGrasp, "Place")
    place.eef = eef
    place.object = object_name
    place.eef_frame = "link6"

    retract = TwistStamped()
    retract.header.frame_id = "link_base"
    retract.twist.linear.z = SPEED
    place.setRetractMotion(retract, MIN, MAX)

    placeMotion = TwistStamped()
    placeMotion.header.frame_id = "link_tcp"
    placeMotion.twist.linear.z = SPEED
    place.setPlaceMotion(placeMotion, MIN, MAX)

    task.add(place)

    print("[INFO] Starting MTC planning...")
    if task.plan():
        print(f"[INFO] Planning succeeded! Found {len(task.solutions)} solution(s)")

        if len(task.solutions) > 0:
            print("[INFO] Publishing solution for visualization...")
            task.publish(task.solutions[0])
            time.sleep(2.0)

            print("[INFO] Executing solution on robot...")
            try:
                result = task.execute(task.solutions[0])
                if result is True or (hasattr(result, 'val') and result.val == 1):
                    print("[INFO] ✓ Execution completed successfully!")
                else:
                    print(f"[WARN] Execution returned: {result} (may still be successful)")
            except Exception as e:
                print(f"[ERROR] ✗ Execution failed with exception: {e}")
        else:
            print("[ERROR] No solutions found to execute!")
    else:
        print("[ERROR] Planning failed!")

    del pipeline
    del planners
    time.sleep(2.0)


if __name__ == "__main__":
    main()
