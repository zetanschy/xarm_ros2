#!/usr/bin/env python3
"""Plomeria del Ejercicio 1. No hace falta tocar nada de aca.

Dos ayudantes:

    cam = MarkerCamera(node)   # ve el marcador ArUco
    arm = ServoArm(node)       # mueve el brazo y el gripper

y con eso el nodo del alumno se ocupa solo del lazo de control.
"""

import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

MARKER_TIMEOUT = 0.5     # s sin ver el marcador -> se considera perdido

GRASP_TCP_Z = 0.070      # altura del TCP para agarrar, en link_base (medida)
KP_Z = 1.2               # ganancia del descenso
GATE_PX = 135.0          # con mas error que esto, el descenso se frena del todo
MAX_SPEED = 0.25         # m/s, tope de cualquier eje
LIFT_SPEED = 0.04        # subir MUCHO mas lento que bajar, o el cubo se cae

# drive_joint: 0 = abierto, 0.85 = cerrado del todo. 0.50 deja la apertura en
# los 40 mm del cubo; pedir mas cierre lo expulsa (probado con 0.60 y 0.85).
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.50

SETTLE_TIME = 2.0        # s esperando a que los dedos cierren
LIFT_TIME = 5.0          # s de ascenso


class MarkerCamera:
    """Detecta el marcador ArUco de la camara de muneca."""

    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()
        # OpenCV 4.11: detectMarkers es un metodo de ArucoDetector. La funcion
        # suelta cv2.aruco.detectMarkers(...) de los tutoriales ya no existe.
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters())
        self.fx = self.cx = self.cy = None
        self._centre = None
        self._stamp = 0.0
        node.create_subscription(CameraInfo, '/wrist_camera/camera_info',
                                 self._on_info, 10)
        node.create_subscription(Image, '/wrist_camera/image_raw',
                                 self._on_image, 10)

    def _on_info(self, msg):
        self.fx, self.cx, self.cy = msg.k[0], msg.k[2], msg.k[5]

    def _on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is None or len(ids) == 0:
            return
        c = corners[0].reshape(4, 2).mean(axis=0)
        self._centre = (float(c[0]), float(c[1]))
        self._stamp = time.time()

    def ready(self):
        """True cuando ya llegaron los intrinsecos de la camara."""
        return self.fx is not None

    def visible(self):
        """True si se esta viendo el marcador ahora mismo."""
        return (self._centre is not None
                and time.time() - self._stamp < MARKER_TIMEOUT)

    def centre(self):
        """(u, v) del centro del marcador, en pixeles."""
        return self._centre


class ServoArm:
    """Mueve el brazo por MoveIt Servo, y maneja el gripper."""

    def __init__(self, node):
        self.node = node
        self.busy = False          # True mientras corre el agarre
        self.done = False          # True cuando ya levanto el cubo: se acabo
        self._phase = None
        self._t0 = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.twist_pub = node.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.gripper = ActionClient(
            node, FollowJointTrajectory,
            '/xarm_gripper_traj_controller/follow_joint_trajectory')
        if not self.gripper.wait_for_server(timeout_sec=10.0):
            node.get_logger().error('el action del gripper no responde')

        self._start_servo()
        self.set_gripper(GRIPPER_OPEN)
        node.create_timer(0.02, self._step_grasp)

    def _start_servo(self):
        """El servo arranca DETENIDO: sin esto los twists no hacen nada."""
        cli = self.node.create_client(Trigger, '/servo_node/start_servo')
        if not cli.wait_for_service(timeout_sec=15.0):
            self.node.get_logger().error('no aparecio /servo_node/start_servo')
            return
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        ok = fut.result() is not None and fut.result().success
        self.node.get_logger().info(f'start_servo: {"OK" if ok else "FALLO"}')

    def move(self, vx=0.0, vy=0.0, vz=0.0):
        """Velocidad en el frame optico de la camara, en m/s.

        +x = derecha de la imagen   +y = abajo   +z = hacia la mesa
        """
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'wrist_camera_optical_frame'
        msg.twist.linear.x = float(np.clip(vx, -MAX_SPEED, MAX_SPEED))
        msg.twist.linear.y = float(np.clip(vy, -MAX_SPEED, MAX_SPEED))
        msg.twist.linear.z = float(np.clip(vz, -MAX_SPEED, MAX_SPEED))
        self.twist_pub.publish(msg)

    def stop(self):
        """Frena el brazo. OJO: dejar de llamar a move() NO lo frena."""
        self.move(0.0, 0.0, 0.0)

    def height(self):
        """Altura del TCP sobre link_base, o None si TF todavia no responde."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'link_base', 'link_tcp', rclpy.time.Time())
            return tf.transform.translation.z
        except Exception:
            return None

    def at_grasp_height(self):
        z = self.height()
        return z is not None and abs(z - GRASP_TCP_Z) < 0.006

    def descend_speed(self, err_px):
        """Velocidad de bajada que toca segun lo que falte para el cubo.

        Se estrangula con el error de centrado: si el marcador esta lejos del
        centro conviene bajar despacio, para no bajar en diagonal.
        """
        z = self.height()
        if z is None:
            return 0.0
        gate = max(0.0, 1.0 - err_px / GATE_PX)
        return float(np.clip(KP_Z * (z - GRASP_TCP_Z) * gate, 0.0, MAX_SPEED))

    def set_gripper(self, position):
        """Manda el goal y vuelve enseguida: esperarlo colgaria el nodo."""
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['drive_joint']
        traj.header.stamp.sec = 0        # 0 = "arranca cuando llegue"
        traj.header.stamp.nanosec = 0
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)]
        pt.time_from_start.sec = 1
        traj.points = [pt]
        goal.trajectory = traj
        self.gripper.send_goal_async(goal)

    def grab_and_lift(self):
        """Cierra el gripper y sube con el cubo. Vuelve enseguida.

        La secuencia corre sola, un pasito por tick, y mientras tanto `busy`
        queda en True. No se hace con time.sleep() porque bloquear el nodo
        congela su reloj y el servo empieza a descartar los comandos.
        """
        if self.busy:
            return
        self.busy = True
        self._phase, self._t0 = 'CLOSING', time.time()
        self.node.get_logger().info('cerrando gripper')

    def _step_grasp(self):
        if not self.busy:
            return
        elapsed = time.time() - self._t0

        if self._phase == 'CLOSING':
            self.stop()
            if elapsed > 0.2:
                self.set_gripper(GRIPPER_CLOSE)
                self._phase, self._t0 = 'SETTLE', time.time()

        elif self._phase == 'SETTLE':
            self.stop()                  # quieto mientras los dedos cierran
            if elapsed > SETTLE_TIME:
                self._phase, self._t0 = 'LIFT', time.time()
                self.node.get_logger().info('subiendo')

        elif self._phase == 'LIFT':
            if elapsed < LIFT_TIME:
                self.move(0.0, 0.0, -LIFT_SPEED)
            else:
                self.stop()
                self.busy = False
                self.done = True
                self._phase = 'DONE'
                self.node.get_logger().info('listo')
