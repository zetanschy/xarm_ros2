#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example: Gemini Detection of Defective Cube

This script demonstrates how to:
1. Read an image from the camera topic
2. Use Gemini Robotics to detect the defective cube
3. Convert pixel coordinates to 3D world coordinates
4. Print the 3D position

Usage:
    ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_gemini_detection use_sim_time:=true
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
from rclpy.duration import Duration
import tf2_ros
import tf_transformations
import json
from PIL import Image as PILImage

# Gemini imports
from google import genai
from google.genai import types
import textwrap

MODEL_ID = "gemini-robotics-er-1.5-preview"

# Pon tu API key de Google AI Studio en la variable de entorno GEMINI_API_KEY:
#     export GEMINI_API_KEY="tu-api-key"
# No la escribas aqui: este archivo esta versionado y la key terminaria en el repo.
API_KEY = os.environ.get("GEMINI_API_KEY", "")


def get_image_from_topic(node: Node, topic: str, timeout_sec: float = 5.0):
    """
    Subscribe once to a ROS2 Image topic and return:
      - cv_bgr (numpy HxWx3)
      - pil_img (PIL RGB)  [same resolution, NO resize]
    """
    bridge = CvBridge()
    holder = {"msg": None}

    def cb(msg: RosImage):
        holder["msg"] = msg

    sub = node.create_subscription(RosImage, topic, cb, 10)

    t0 = time.time()
    while rclpy.ok() and holder["msg"] is None and (time.time() - t0) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_subscription(sub)

    if holder["msg"] is None:
        raise TimeoutError(f"Timed out waiting for image on topic '{topic}'")

    cv_bgr = bridge.imgmsg_to_cv2(holder["msg"], desired_encoding="bgr8")
    cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
    pil_img = PILImage.fromarray(cv_rgb)

    return cv_bgr, pil_img


def pixel_to_world(
    node,
    tf_buffer: tf2_ros.Buffer,
    u: float,
    v: float,
    base_frame: str = "link_base",
    camera_frame: str = "overhead_camera_link",
    Z_cam: float = -0.8,       
    fx: float = 585.756,
    fy: float = 585.756,
    cx: float = 320.0,
    cy: float = 240.0,
    timeout_sec: float = 2.0,
):
    """
    Convert pixel coordinates to 3D world coordinates.
    
    Replicates ColorDetector math exactly:
      Y = (u - cx) * Z / fx
      X = (v - cy) * Z / fy
      Z = Z_cam
    Then transforms pt_cam -> base_frame using TF lookup_transform(base, camera).

    Returns: (x_base, y_base, z_base)
    """
    t0 = time.time()
    last_err = None

    # ---- same math as detector ----
    Y_cam = (u - cx) * Z_cam / fx
    X_cam = (v - cy) * Z_cam / fy
    pt_cam = np.array([X_cam, Y_cam, Z_cam, 1.0], dtype=float)

    while (time.time() - t0) < timeout_sec:
        try:
            rclpy.spin_once(node, timeout_sec=0.05)

            t = tf_buffer.lookup_transform(
                base_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )

            trans = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ], dtype=float)

            quat = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]

            T = tf_transformations.quaternion_matrix(quat)
            T[:3, 3] = trans

            pt_base = T @ pt_cam
            return float(pt_base[0]), float(pt_base[1]), float(pt_base[2])

        except Exception as e:
            last_err = e
            time.sleep(0.02)

    raise TimeoutError(f"pixel_to_world failed: {last_err}")


def parse_json(json_output):
    """Parsing out the markdown fencing"""
    lines = json_output.splitlines()
    for i, line in enumerate(lines):
        if line == "```json":
            # Remove everything before "```json"
            json_output = "\n".join(lines[i + 1 :])
            # Remove everything after the closing "```"
            json_output = json_output.split("```")[0]
            break  # Exit the loop once "```json" is found
    return json_output


def call_gemini_robotics_er(client, img, prompt, config=None):
    default_config = types.GenerateContentConfig(
        temperature=0.5,
        thinking_config=types.ThinkingConfig(thinking_budget=0)
    )

    if config is None:
        config = default_config

    image_response = client.models.generate_content(
          model=MODEL_ID,
          contents=[img, prompt],
          config=config,
    )

    print(image_response.text)
    return parse_json(image_response.text)


def gemini_point_to_norm_xy(json_text: str):
    """
    Expects: [{"point":[y,x], "label":"..."}]
    Returns: (x_norm, y_norm) in [0,1]
    """
    cleaned = json_text.strip().replace("```json", "").replace("```", "").strip()
    data = json.loads(cleaned)
    if not isinstance(data, list) or len(data) == 0:
        raise ValueError(f"Gemini output is not a non-empty list: {data}")

    pt = data[0].get("point", None)
    if pt is None or not (isinstance(pt, list) and len(pt) == 2):
        raise ValueError(f"Missing/invalid point in first item: {data[0]}")

    y_0_1000 = float(pt[0])
    x_0_1000 = float(pt[1])

    x_norm = max(0.0, min(1.0, x_0_1000 / 1000.0))
    y_norm = max(0.0, min(1.0, y_0_1000 / 1000.0))
    return x_norm, y_norm


def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    node = Node('example_gemini_detection')
    
    node.get_logger().info("="*60)
    node.get_logger().info("Gemini Detection Example: Defective Cube")
    node.get_logger().info("="*60)
    
    try:
        # 1. Get image from camera
        node.get_logger().info("Step 1: Getting image from /camera/image_raw...")
        cv_bgr, img_pil = get_image_from_topic(node, "/camera/image_raw")
        node.get_logger().info(f"✓ Image received: {cv_bgr.shape[1]}x{cv_bgr.shape[0]} pixels")
        
        # 2. Call Gemini to detect defective cube
        node.get_logger().info("Step 2: Calling Gemini to detect defective cube...")
        prompt = textwrap.dedent("""\
            Point to the DEFECTIVE cube that I need to remove.
            The defective cube is a red cube with a warped base or defects.

            The answer should follow the JSON format:
            [{"point": <point>, "label": <label>}, ...]

            The points are in [y, x] format normalized to 0-1000.
        """)
        
        config = types.GenerateContentConfig(
            temperature=0.5,
            thinking_config=types.ThinkingConfig(thinking_budget=1024),
        )
        
        start_time = time.time()
        if not API_KEY:
            node.get_logger().error(
                "GEMINI_API_KEY no esta definida. Corre: export GEMINI_API_KEY=\"tu-api-key\"")
            return
        client = genai.Client(api_key=API_KEY)
        json_output = call_gemini_robotics_er(client, img_pil, prompt, config)
        processing_time = time.time() - start_time
        node.get_logger().info(f"✓ Gemini processing time: {processing_time:.4f} seconds")
        
        # 3. Parse Gemini response
        node.get_logger().info("Step 3: Parsing Gemini response...")
        x_start_norm, y_start_norm = gemini_point_to_norm_xy(json_output)
        h, w = cv_bgr.shape[:2]
        u = x_start_norm * (w - 1)
        v = y_start_norm * (h - 1)
        node.get_logger().info(f"✓ Pixel coordinates: u={u:.1f}, v={v:.1f}")
        
        # 4. Convert to 3D world coordinates
        node.get_logger().info("Step 4: Converting pixel coordinates to 3D world coordinates...")
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)
        time.sleep(1.0)  # Wait for TF buffer to fill
        
        world_pos = pixel_to_world(node, tf_buffer, u, v)
        x_3d, y_3d, z_3d = world_pos
        z_3d = 0.0  # Cube sits on ground
        
        # 5. Print results
        node.get_logger().info("="*60)
        node.get_logger().info("RESULTS:")
        node.get_logger().info("="*60)
        node.get_logger().info(f"Defective Cube 3D Position (in 'link_base' frame):")
        node.get_logger().info(f"  X: {x_3d:.4f} m")
        node.get_logger().info(f"  Y: {y_3d:.4f} m")
        node.get_logger().info(f"  Z: {z_3d:.4f} m")
        node.get_logger().info("="*60)
        
        # Also print to console for easy copy-paste
        print("\n" + "="*60)
        print("DEFECTIVE CUBE 3D POSITION:")
        print(f"  x = {x_3d:.4f}")
        print(f"  y = {y_3d:.4f}")
        print(f"  z = {z_3d:.4f}")
        print("="*60 + "\n")
        
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

