#!/usr/bin/env python3
"""
Demo de Hybrid Planning con el xArm6 - Clase 5.

Que se ve en la demo
--------------------
1. Se agrega una pared al planning scene.
2. Se manda UNA meta al hybrid planner. El global planner (OMPL) resuelve el
   camino completo y el local planner empieza a ejecutarlo a 50 Hz.
3. A mitad del movimiento aparece una segunda pared cruzando el camino ya
   planificado.
4. El local planner detecta que la trayectoria dejo de ser valida y avisa. El
   planner logic plugin (ReplanInvalidatedTrajectory) le pide al global planner
   un plan nuevo, y la ejecucion sigue sin volver a cero.

Ese paso 4 es la diferencia con lo que se vio en las clases anteriores: con
move_group, si el mundo cambia despues de planificar, la trayectoria se ejecuta
igual (o se aborta). Aca se replanifica en caliente.

Uso
---
    # Terminal 1: simulacion
    ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \\
        world:=table_gz.world add_gripper:=true

    # Terminal 2: hybrid planner + esta demo
    ros2 launch xarm_scripts hybrid_planning_xarm6.launch.py
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import HybridPlanner
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionSequenceItem,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GROUP = 'xarm6'
BASE_FRAME = 'link_base'
JOINT_NAMES = [f'joint{i}' for i in range(1, 7)]

# Meta en espacio articular. Con esta meta el TCP va de (+0.545, 0.000, 0.479)
# a (+0.084, +0.486, +0.369): un barrido amplio por el cuadrante +x/+y, que es
# lo que hace que valga la pena meterle un obstaculo en el camino.
GOAL_JOINTS = [1.4, -0.5, -0.8, 0.0, 0.6, 0.0]

# Pose de arranque. La demo primero manda el brazo aca y despues hace el barrido,
# para que cada corrida en clase salga igual. Sin esto, si el brazo quedo cerca
# de la meta de la corrida anterior, el movimiento dura 150 ms, nada se alcanza a
# ver y la pared sorpresa nunca llega a entrar.
START_JOINTS = [0.0, -0.5, -0.75, 0.0, 0.0, 0.0]

# Pared baja que ya esta cuando se planifica. Queda en el camino del barrido,
# asi que el global planner tiene que pasar por arriba desde el primer plan.
WALL_STATIC = {
    'id': 'pared_estatica',
    'size': [0.05, 0.5, 0.25],
    'pos': (0.35, 0.25, 0.10),
}

# Pared que aparece a mitad del movimiento. Ocupa la parte alta cerca de la meta,
# asi que invalida lo que queda de la trayectoria pero deja una salida por abajo:
# si tapara todo, el local planner se quedaria frenado para siempre y no se veria
# el replan.
# El TCP recorre este camino (FK sobre el camino articular START -> GOAL):
#     0%  (+0.539, +0.000, +0.495)
#    35%  (+0.470, +0.250, +0.448)   <- aca se inyecta la pared
#    60%  (+0.350, +0.380, +0.420)   <- aca la pared la bloquea
#   100%  (+0.084, +0.486, +0.369)   <- meta, lejos de la pared
# La pared va al 60%: corta el arco directo pero deja salida por arriba, y no
# toca la zona de la meta. Si se la pone encima de la meta, el global planner
# devuelve "failed to find a solution" y no se ve el replan.
# Caja chica, no una pared: si el obstaculo es grande, o cae encima de donde ya
# esta el brazo, el estado inicial queda en colision y el global planner devuelve
# "failed to find a solution" en vez de replanificar. Va cerca del 75% del camino
# (TCP +0.255, +0.444, +0.397), bien por delante del punto de inyeccion (35%,
# TCP +0.470, +0.250, +0.448), y es lo bastante chica para que quede vuelta.
WALL_SURPRISE = {
    'id': 'obstaculo_sorpresa',
    'size': [0.06, 0.14, 0.18],
    'pos': (0.26, 0.44, 0.40),
}

# Fraccion del camino articular que se recorre antes de meter la pared sorpresa.
# NO se puede usar el primer feedback: llega ~1 ms despues de aceptar la meta,
# cuando el brazo todavia no se movio, y entonces el local planner frena de una
# y el manager entra en un bucle de replanificacion sin haber ejecutado nada.
SURPRISE_AT_PROGRESS = 0.35


def make_box(spec, frame_id=BASE_FRAME):
    """Arma un CollisionObject de tipo caja a partir de un dict de spec."""
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = spec['id']

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(spec['size'])

    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x, pose.position.y, pose.position.z = spec['pos']

    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


def remove_box(object_id, frame_id=BASE_FRAME):
    """CollisionObject que borra un objeto del scene por id."""
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.operation = CollisionObject.REMOVE
    return obj


class HybridPlanningDemo(Node):
    """Manda una meta al hybrid planner y cambia el mundo a mitad de ejecucion."""

    def __init__(self):
        super().__init__('hybrid_planning_demo')

        # El nombre de la accion viene del yaml comun, el mismo que usan los tres
        # componentes. Se declara con un default para poder correr el nodo suelto.
        self.declare_parameter(
            'hybrid_planning_action_name',
            '/xarm6/hybrid_planning/run_hybrid_planning')
        action_name = self.get_parameter(
            'hybrid_planning_action_name').get_parameter_value().string_value

        self.get_logger().info(f'Accion del hybrid planner: {action_name}')
        self.client = ActionClient(self, HybridPlanner, action_name)

        self.scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')

        # Publisher directo al controlador, para el reset sin planificar.
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/xarm6_traj_controller/joint_trajectory', 10)

        self.surprise_sent = False
        self.surprise_enabled = False
        self.done = False
        self.result_code = None
        self.goal_joints = START_JOINTS

        # Progreso del movimiento, para saber cuando meter la pared sorpresa.
        self.joint_positions = {}
        self.initial_distance = None
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)

    # ------------------------------------------------------------------ #
    # Planning scene
    # ------------------------------------------------------------------ #
    def apply_objects(self, objects):
        """Aplica objetos al scene de move_group (y por lo tanto a RViz)."""
        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/apply_planning_scene no disponible.')
            return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = list(objects)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        return bool(response and response.success)

    # ------------------------------------------------------------------ #
    # Meta
    # ------------------------------------------------------------------ #
    def build_goal(self, joints):
        """Arma el HybridPlanner.Goal con una sola meta en espacio articular."""
        request = MotionPlanRequest()
        request.group_name = GROUP
        request.pipeline_id = 'ompl'
        # Los nombres vienen de xarm_moveit_config/config/xarm6/ompl_planning.yaml,
        # que usa 'RRTConnect' y no 'RRTConnectkConfigDefault'.
        request.planner_id = 'RRTConnect'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.4
        request.max_acceleration_scaling_factor = 0.4

        constraints = Constraints()
        for name, position in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(position)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.goal_constraints.append(constraints)

        item = MotionSequenceItem()
        item.req = request
        item.blend_radius = 0.0  # una sola meta, no hay nada que mezclar

        goal = HybridPlanner.Goal()
        goal.planning_group = GROUP
        goal.motion_sequence.items.append(item)
        return goal

    # ------------------------------------------------------------------ #
    # Callbacks de la accion
    # ------------------------------------------------------------------ #
    def on_joint_states(self, msg):
        """Sigue el progreso y mete la pared sorpresa a mitad de camino."""
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

        distance = self.distance_to_goal()
        if distance is None:
            return

        if self.initial_distance is None:
            self.initial_distance = distance
            return

        if (not self.surprise_enabled or self.surprise_sent
                or self.initial_distance < 1e-3):
            return

        progress = 1.0 - (distance / self.initial_distance)
        if progress >= SURPRISE_AT_PROGRESS:
            self.surprise_sent = True
            self.get_logger().warning(
                f'>>> {progress * 100:.0f}% del camino recorrido: aparece '
                '"pared_sorpresa" cruzando lo que queda de la trayectoria. El '
                'local planner deberia invalidarla y el manager pedirle un plan '
                'nuevo al global planner, sin volver a empezar.')
            self.apply_objects([make_box(WALL_SURPRISE)])

    def distance_to_goal(self):
        """Distancia en espacio articular hasta la meta, o None si falta info."""
        if not all(name in self.joint_positions for name in JOINT_NAMES):
            return None
        return sum(
            abs(self.joint_positions[name] - goal)
            for name, goal in zip(JOINT_NAMES, self.goal_joints)
        )

    def on_feedback(self, msg):
        self.get_logger().info(f'[feedback] {msg.feedback.feedback}')

    def on_result(self, future):
        result = future.result()
        self.result_code = result.result.error_code.val
        message = result.result.error_message
        if self.result_code == 1:
            self.get_logger().info('Hybrid planning termino OK.')
        else:
            self.get_logger().error(
                f'Hybrid planning fallo: error_code={self.result_code} "{message}"')
        self.done = True

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('El hybrid planning manager rechazo la meta.')
            self.done = True
            return
        self.get_logger().info('Meta aceptada, ejecutando...')
        handle.get_result_async().add_done_callback(self.on_result)

    # ------------------------------------------------------------------ #
    def reset_without_planning(self, joints, min_duration_sec=5.0):
        """
        Manda el brazo a una pose conocida comandando el controlador directo.

        No pasa por el planner a proposito. Si una corrida anterior dejo el brazo
        en una configuracion rara o fuera de limites, el global planner responde
        "failed to find a solution" y ni el paso de ir al inicio funciona. Con un
        JointTrajectory al controlador se sale de ahi siempre.
        """
        # La duracion se escala con la distancia: desde una pose lejana, una
        # trayectoria fija de 5 s no alcanza y el reset se queda a mitad.
        self.goal_joints = list(joints)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.distance_to_goal() is not None:
                break
        distance = self.distance_to_goal() or 0.0
        duration_sec = max(min_duration_sec, distance * 2.0)

        message = JointTrajectory()
        message.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in joints]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        message.points.append(point)

        self.get_logger().info(
            f'Reset directo al controlador: {[round(v, 3) for v in joints]} '
            f'(distancia {distance:.2f} rad, {duration_sec:.1f} s)')
        # Se repite unas veces porque es un topico, no una accion: si el
        # controlador todavia no se suscribio, el primer mensaje se pierde.
        for _ in range(5):
            self.traj_pub.publish(message)
            time.sleep(0.2)

        deadline = time.time() + duration_sec + 5.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            distance = self.distance_to_goal()
            if distance is not None and distance < 0.05:
                self.get_logger().info('Reset completo.')
                return True
        self.get_logger().warning(
            'El reset no llego a la tolerancia; se sigue igual.')
        return False

    def send_goal_and_wait(self, joints, allow_surprise, timeout_sec=120.0):
        """Manda una meta al hybrid planner y espera el resultado."""
        self.goal_joints = list(joints)
        self.surprise_enabled = allow_surprise
        self.surprise_sent = False
        self.done = False
        self.result_code = None

        # Linea base del progreso justo antes de arrancar.
        self.initial_distance = None
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            distance = self.distance_to_goal()
            if distance is not None:
                self.initial_distance = distance
                break
        if self.initial_distance is None:
            self.get_logger().error('No llegaron /joint_states.')
            return False

        self.get_logger().info(
            f'Meta articular: {[round(v, 3) for v in joints]} '
            f'(distancia inicial {self.initial_distance:.3f} rad)')

        send_future = self.client.send_goal_async(
            self.build_goal(joints), feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)

        deadline = time.time() + timeout_sec
        while rclpy.ok() and not self.done and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

        if not self.done:
            self.get_logger().error('Timeout esperando el resultado.')
            return False
        return self.result_code == 1

    def run(self):
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(
                'No aparecio el action server del hybrid planner. Revisa que '
                'hybrid_planning_xarm6.launch.py haya levantado el contenedor.')
            return False

        # Paso 0: escena limpia y brazo en una pose conocida, para que la demo
        # se vea igual en cada corrida.
        self.get_logger().info('--- Paso 0: limpiando escena y yendo al inicio ---')
        self.apply_objects([
            remove_box(WALL_STATIC['id']),
            remove_box(WALL_SURPRISE['id']),
        ])
        time.sleep(1.0)
        # Reset sin planificar: robusto incluso si la corrida anterior dejo el
        # brazo en una pose invalida.
        self.reset_without_planning(START_JOINTS)
        time.sleep(1.0)

        # Paso 1: obstaculo que ya esta cuando se planifica.
        self.get_logger().info(
            '--- Paso 1: agregando "pared_estatica" (el global planner la '
            'esquiva desde el primer plan) ---')
        if not self.apply_objects([make_box(WALL_STATIC)]):
            self.get_logger().error('No se pudo aplicar el objeto al scene.')
            return False
        time.sleep(2.0)  # que el scene se propague a los dos planners

        # Paso 2: barrido con la pared sorpresa a mitad de camino.
        self.get_logger().info(
            '--- Paso 2: barrido hacia la meta. A mitad de camino aparece '
            '"pared_sorpresa" ---')
        ok = self.send_goal_and_wait(GOAL_JOINTS, allow_surprise=True)
        if self.surprise_sent:
            self.get_logger().info(
                'La pared sorpresa se inyecto durante la ejecucion: eso es lo '
                'que dispara el replan del global planner.')
        else:
            self.get_logger().warning(
                'La pared sorpresa NO se inyecto (el movimiento termino antes de '
                f'llegar al {SURPRISE_AT_PROGRESS * 100:.0f}% del camino).')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = HybridPlanningDemo()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
