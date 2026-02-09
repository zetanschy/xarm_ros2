#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt Task Constructor (MTC) Pick and Place Demo for xArm6

Closely follows the reference:
  moveit_task_constructor/demo/scripts/pickplace.py
but adapted for xarm6 + xarm_gripper.

Kinematic chain (URDF):
  link6 → link_eef (identity) → xarm_gripper_base_link (identity) → link_tcp (z+0.172)
SRDF end-effector parent_link = link_tcp

Frame mapping from Panda reference:
  panda_arm   → xarm6
  hand        → xarm_gripper
  panda_hand  → link_tcp    (EEF body frame)
  panda_link8 → link6       (last arm link)
  world       → link_base   (fixed frame)
"""

import rclpy
from rclpy.node import Node as RclpyNode

import rclcpp
from moveit.task_constructor import core, stages
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from moveit_msgs.msg import (
    Constraints, OrientationConstraint,
    CollisionObject, PlanningScene,
)
from shape_msgs.msg import SolidPrimitive
import math
import time


def publish_collision_object(object_name, pose_stamped, dimensions):
    """Publish a collision object (box) to the planning scene via rclpy."""
    rclpy.init()
    tmp_node = RclpyNode("_scene_setup")

    pub = tmp_node.create_publisher(PlanningScene, "/planning_scene", 10)
    time.sleep(1.0)  # Wait for publisher discovery

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

    pub.publish(ps)
    tmp_node.get_logger().info(
        f"Published collision object '{object_name}' at "
        f"({pose_stamped.pose.position.x}, {pose_stamped.pose.position.y}, "
        f"{pose_stamped.pose.position.z}) in '{pose_stamped.header.frame_id}'"
    )
    time.sleep(0.5)

    tmp_node.destroy_node()
    rclpy.shutdown()


def main():
    # ── Robot parameters (xarm6 equivalents of panda_arm / hand) ──
    arm = "xarm6"
    eef = "xarm_gripper"

    # ── Object parameters (matching pick_and_place.py) ──
    object_name = "object"

    # Pick / Place positions from pick_and_place.py
    X_LEVEL = -0.3
    START_Y = -0.4
    GOAL_Y  =  0.4

    # Object dimensions: same box as pick_and_place.py [0.05, 0.05, 0.1]
    object_size = [0.05, 0.05, 0.1]

    # Object pose at the pick location (center of box on the surface)
    objectPose = PoseStamped()
    objectPose.header.frame_id = "link_base"
    objectPose.pose.orientation.w = 1.0
    objectPose.pose.position.x = X_LEVEL   # -0.3
    objectPose.pose.position.y = START_Y   # -0.4
    objectPose.pose.position.z = object_size[2] / 2.0  # half-height above ground

    # ── 1. Add collision object to the planning scene ──
    publish_collision_object(object_name, objectPose, object_size)

    # ── 2. MTC task (via rclcpp, same as reference) ──
    rclcpp.init()
    # Use NodeOptions to automatically declare parameters from overrides
    # This ensures time parameterization settings are properly loaded
    node_options = rclcpp.NodeOptions()
    node_options.automatically_declare_parameters_from_overrides = True
    node = rclcpp.Node("mtc_pickplace", node_options)

    # Create a task
    task = core.Task()
    task.name = "pick + place"
    task.loadRobotModel(node)

    # Start with the current state
    task.add(stages.CurrentState("current"))

    # ── Planner ──
    pipeline = core.PipelinePlanner(node)
    pipeline.planner = "RRTConnectkConfigDefault"
    planners = [(arm, pipeline)]

    # Connect current state → grasp approach
    task.add(stages.Connect("connect", planners))

    # ── Generate Grasp Pose ──
    grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
    grasp_generator.angle_delta = math.pi / 2   # 4 candidates — axis-aligned grasps
    grasp_generator.pregrasp = "open"
    grasp_generator.grasp = "grasp"  # partial close (drive_joint=0.5), matching pick_and_place.py
    grasp_generator.setMonitoredStage(task["current"])

    # ── SimpleGrasp (IK + finger closing) ──
    simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")

    # IK frame at the TCP, with 180° rotation around X for top-down grasp
    # (same concept as reference: panda_hand + orientation.x = 1.0)
    ik_frame = PoseStamped()
    ik_frame.header.frame_id = "link_tcp"       # xarm TCP frame
    ik_frame.pose.position.z = 0.0              # already at fingertip center
    ik_frame.pose.orientation.x = 1.0           # 180° around X → grasp from above
    ik_frame.pose.orientation.w = 0.0
    simpleGrasp.setIKFrame(ik_frame)

    # ── Pick (approach + grasp + lift) ──
    pick = stages.Pick(simpleGrasp, "Pick")
    pick.eef = eef
    pick.object = object_name

    # Approach: move down in world frame (same as reference)
    approach = TwistStamped()
    approach.header.frame_id = "link_base"
    approach.twist.linear.z = -1.0
    pick.setApproachMotion(approach, 0.03, 0.15)

    # Lift: move in -Z of EEF frame → upward when gripper points down
    # (same as reference: panda_hand, z = -1.0)
    lift = TwistStamped()
    lift.header.frame_id = "link_tcp"
    lift.twist.linear.z = -1.0
    pick.setLiftMotion(lift, 0.03, 0.15)

    task.add(pick)

    # ── Orientation constraint: keep object upright during transport ──
    oc = OrientationConstraint()
    oc.parameterization = oc.ROTATION_VECTOR
    oc.header.frame_id = "link_base"
    oc.link_name = object_name
    oc.orientation.w = 1.0
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = math.pi
    oc.weight = 1.0

    constraints = Constraints()
    constraints.name = "object:upright"
    constraints.orientation_constraints.append(oc)

    # Connect Pick → Place with upright constraint
    con = stages.Connect("connect2", planners)
    con.path_constraints = constraints
    task.add(con)

    # ── Place pose (matching pick_and_place.py) ──
    placePose = PoseStamped()
    placePose.header.frame_id = "link_base"
    placePose.pose.orientation.w = 1.0
    placePose.pose.position.x = X_LEVEL    # -0.3
    placePose.pose.position.y = GOAL_Y     # 0.4
    placePose.pose.position.z = object_size[2] / 2.0  # same height as pick

    # Generate place poses
    place_generator = stages.GeneratePlacePose("Generate Place Pose")
    place_generator.setMonitoredStage(task["Pick"])
    place_generator.object = object_name
    place_generator.pose = placePose

    # SimpleUnGrasp (release at Cartesian pose)
    simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp")

    # Place (place + ungrasp + retract)
    place = stages.Place(simpleUnGrasp, "Place")
    place.eef = eef
    place.object = object_name
    place.eef_frame = "link6"     # last arm link (equiv. to panda_link8)

    # Retract: move up in world frame (same as reference)
    retract = TwistStamped()
    retract.header.frame_id = "link_base"
    retract.twist.linear.z = 1.0
    place.setRetractMotion(retract, 0.03, 0.15)

    # PlaceMotion: move in +Z of EEF frame → downward when gripper points down
    # (same as reference: panda_hand, z = 1.0)
    placeMotion = TwistStamped()
    placeMotion.header.frame_id = "link_tcp"
    placeMotion.twist.linear.z = 1.0
    place.setPlaceMotion(placeMotion, 0.03, 0.15)

    task.add(place)

    # ── 3. Plan & publish ──
    if task.plan():
        task.publish(task.solutions[0])
        print("[INFO] Planning succeeded! Solution published to RViz.")
    else:
        print("[ERROR] Planning failed!")

    # Avoid ClassLoader warning
    del pipeline
    del planners

    # Keep alive so you can inspect solutions in RViz
    time.sleep(3600)


if __name__ == "__main__":
    main()
