#!/usr/bin/env python3
"""
Demo de Hybrid Planning con el xArm6 - Clase 5.

Que se ve en la demo
--------------------
1. Se agrega una pared al planning scene.
2. Se manda UNA meta al hybrid planner. El global planner (OMPL) resuelve el
   camino completo y el local planner empieza a ejecutarlo a 50 Hz.
3. En cuanto sale esa primera solucion global aparece una placa nueva cruzando
   el camino ya planificado (y se borra la primera, para que la escena no
   acumule obstaculos).
4. El local planner detecta que la trayectoria dejo de ser valida y FRENA el
   brazo en el lugar (stop_before_collision). El planner logic plugin
   (xarm_hybrid_planning/ReplanWhenIdle) le pide al global planner un plan nuevo.
5. El plan nuevo RODEA la placa que acaba de aparecer, el local planner lo
   engancha en el waypoint mas cercano al estado real del brazo, y la ejecucion
   sigue hasta la meta sin volver a cero. Medido: cierra en 3 replanificaciones,
   5 de 5 corridas, con 0.022 rad de error final.

Los pasos 4 y 5 son la diferencia con lo que se vio en las clases anteriores: con
move_group, si el mundo cambia despues de planificar, la trayectoria se ejecuta
igual (o se aborta). Aca se frena, se replanifica en caliente, y se esquiva.

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
    MotionPlanResponse,
    MotionSequenceItem,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray

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

# Los obstaculos son PLACAS FINAS (1-2 cm), no cajas macizas. El demo de MoveIt
# usa {0.5, 0.8, 0.01} y {1.0, 0.4, 0.01}, y la razon es practica: una placa
# invalida cualquier trayectoria que la cruce, pero casi no le quita espacio libre
# al brazo, asi que siempre queda un camino alternativo. Con cajas macizas el
# replan se queda sin solucion y todo aborta.

# Placa que ya esta cuando se planifica: el global planner la esquiva de entrada.
WALL_STATIC = {
    'id': 'placa_estatica',
    'size': [0.40, 0.02, 0.30],
    'pos': (0.35, 0.10, 0.25),
}

# Placa que aparece cuando sale la primera solucion global. Se agrega a la vez que
# se BORRA la estatica, como en el demo de MoveIt: la escena no acumula
# obstaculos, y el brazo siempre tiene por donde pasar.
#
# UNA sola placa, no dos como el demo de MoveIt. Con dos, el problema que le queda
# al global planner es lo bastante cerrado como para que muchas soluciones de
# RRTConnect no sean seguibles por ForwardTrajectory, y la demo se vuelve una
# loteria: a veces sale en 1-4 replanificaciones y a veces gasta los reintentos.
# Con una placa el desvio es amplio y cualquier solucion sirve.
WALLS_SURPRISE = [
    {
        'id': 'placa_sorpresa',
        'size': [0.02, 0.35, 0.30],
        'pos': (0.30, 0.32, 0.30),
    },
]


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

        # Publisher directo al controlador de posicion, para el reset sin
        # planificar. Es el mismo topico que usa el local planner.
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/xarm6_joint_group_position_controller/commands', 10)

        self.surprise_sent = False
        self.surprise_enabled = False
        self.done = False
        self.result_code = None
        self.goal_joints = START_JOINTS

        self.joint_positions = {}
        self.initial_distance = None
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10)

        # El swap de obstaculos se dispara cuando el global planner PUBLICA una
        # solucion, igual que el demo de MoveIt (que se suscribe a
        # 'global_trajectory'). Es mejor que disparar por progreso: pasa una vez
        # por ciclo de planificacion, y como las operaciones son idempotentes la
        # escena deja de cambiar despues del primer swap, asi que el replan
        # siguiente converge en vez de invalidarse otra vez.
        self.create_subscription(
            MotionPlanResponse, 'global_trajectory', self.on_global_solution, 10)

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
        # Estos dos numeros son la diferencia entre que el replan funcione y que
        # todo aborte, y no es obvio por que.
        #
        # El local planner reporta COLLISION_AHEAD cada ~80 ms mientras el brazo
        # siga frenado (forward_trajectory.cpp manda el evento una vez por
        # trayectoria, pero resetea el flag en cuanto llega una solucion global
        # nueva). Cada evento es un pedido de replanificacion.
        #
        # Con el ReplanInvalidatedTrajectory de MoveIt, si el global planner
        # todavia estaba pensando cuando llegaba el evento siguiente, la meta
        # anterior se ABORTABA, y el plugin devolvia FAILURE ante
        # 'Global planning action aborted': moria todo el hybrid planning. Con
        # allowed_planning_time = 5.0 eso pasaba siempre. ReplanWhenIdle no pide
        # un plan si ya hay uno en vuelo, asi que ya no depende de esto; el 0.5
        # se deja igual porque RRTConnect resuelve esto en decenas de ms y no
        # hay razon para darle mas.
        #
        # Con 0.5 s y un solo intento, cada replan termina (RRTConnect resuelve
        # esto en decenas de ms) antes de que llegue el evento siguiente.
        request.num_planning_attempts = 1
        request.allowed_planning_time = 0.5
        # 0.4 y no menos, a proposito. Bajarlo suaviza el movimiento pero ROMPE
        # la demo: con 0.15 el siguiente waypoint queda siempre tan cerca del
        # estado actual que isPathValid nunca lo encuentra en colision, asi que
        # stop_before_collision no dispara nunca (medido: 0 "Collision ahead" en
        # dos corridas) y el brazo llega a la meta sin reaccionar a nada. Ver
        # HYBRID_PLANNING.md, "Por que el movimiento se ve raro".
        request.max_velocity_scaling_factor = 0.4
        request.max_acceleration_scaling_factor = 0.4

        # No se define workspace_parameters, asi que OMPL avisa "It looks like the
        # planning volume was not specified" y usa su espacio por defecto. Se probo
        # acotarlo: el aviso desaparece pero no cambio el comportamiento de la demo
        # (lo que la rompia era el escalado de velocidad, no esto). Se deja sin
        # definir para no agregar numeros que no hacen falta.

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
        """Solo lleva el estado articular, para medir progreso y el reset."""
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def on_global_solution(self, msg):
        """
        Llega una solucion global nueva: es el momento de cambiar la escena.

        Igual que el demo de MoveIt. Se BORRA la placa estatica y se agregan las
        dos placas sorpresa en la misma actualizacion, para que la escena no
        acumule obstaculos. Como las operaciones son idempotentes, a partir del
        segundo aviso la escena ya no cambia y el replan converge.
        """
        if not self.surprise_enabled or self.surprise_sent:
            return
        self.surprise_sent = True
        self.get_logger().warning(
            '>>> Salio la primera solucion global. Ahora cambia la escena: se '
            'borra "placa_estatica" y aparecen dos placas nuevas cruzando la '
            'trayectoria. El local planner deberia invalidarla y el manager '
            'pedirle un plan nuevo al global planner, sin volver a empezar.')
        self.apply_objects(
            [remove_box(WALL_STATIC['id'])]
            + [make_box(w) for w in WALLS_SURPRISE])

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
        elif self.surprise_sent:
            # Con ReplanWhenIdle + SceneSyncingPipeline esto ya NO es lo esperado:
            # medido, el ciclo cierra 5 de 5. Si aparece, es una falla de verdad.
            self.get_logger().error(
                f'Hybrid planning termino con error_code={self.result_code} '
                f'"{message}" DESPUES de inyectar la placa sorpresa.')
            self.get_logger().error(
                'El local planner detecto el obstaculo y freno (eso esta arriba '
                'en el log), pero la replanificacion no llego a la meta. Lo mas '
                'probable es que el brazo arrancara de una pose invalida de una '
                'corrida anterior. Ver HYBRID_PLANNING.md, "Relanzar el ejemplo".')
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
    def reset_without_planning(self, joints, ramp_sec=3.0, rate_hz=50.0):
        """
        Manda el brazo a una pose conocida comandando el controlador de posicion.

        No pasa por el planner a proposito: si una corrida anterior dejo el brazo
        en una configuracion rara, el global planner responde "failed to find a
        solution" y ni el paso de ir al inicio funciona.

        Se hace en rampa y no de un salto porque el JointGroupPositionController
        escribe la posicion tal cual, sin interpolar: un salto grande sacude el
        modelo en Gazebo.
        """
        self.goal_joints = list(joints)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.distance_to_goal() is not None:
                break
        if not all(n in self.joint_positions for n in JOINT_NAMES):
            self.get_logger().error('No llegaron /joint_states.')
            return False

        start = [self.joint_positions[n] for n in JOINT_NAMES]
        target = [float(v) for v in joints]
        steps = max(1, int(ramp_sec * rate_hz))
        self.get_logger().info(
            f'Reset por rampa al controlador de posicion: '
            f'{[round(v, 3) for v in target]} en {ramp_sec:.1f} s')

        for i in range(1, steps + 1):
            f = i / steps
            message = Float64MultiArray()
            message.data = [a + (b - a) * f for a, b in zip(start, target)]
            self.command_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

        deadline = time.time() + 3.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            distance = self.distance_to_goal()
            if distance is not None and distance < 0.05:
                self.get_logger().info('Reset completo.')
                return True
        self.get_logger().warning(
            f'El reset quedo a {self.distance_to_goal():.3f} rad de la pose de '
            'inicio; se sigue igual.')
        return True

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
        self.apply_objects(
            [remove_box(WALL_STATIC['id'])]
            + [remove_box(w['id']) for w in WALLS_SURPRISE])
        time.sleep(1.0)
        # Reset sin planificar: robusto incluso si la corrida anterior dejo el
        # brazo en una pose invalida.
        if not self.reset_without_planning(START_JOINTS):
            self.get_logger().error(
                'El brazo no volvio a la pose de inicio. Si una corrida anterior '
                'lo dejo clavado contra sus limites articulares, ningun comando '
                'de posicion lo saca: hay que reiniciar Gazebo. Ver '
                'HYBRID_PLANNING.md, seccion "Reiniciar entre corridas".')
            return False
        time.sleep(1.0)

        # Paso 1: obstaculo que ya esta cuando se planifica.
        self.get_logger().info(
            '--- Paso 1: agregando "placa_estatica" (el global planner la '
            'esquiva desde el primer plan) ---')
        if not self.apply_objects([make_box(WALL_STATIC)]):
            self.get_logger().error('No se pudo aplicar el objeto al scene.')
            return False
        time.sleep(2.0)  # que el scene se propague a los dos planners

        # Paso 2: barrido con la pared sorpresa a mitad de camino.
        self.get_logger().info(
            '--- Paso 2: barrido hacia la meta. Al salir la primera solucion '
            'global cambia la escena ---')
        ok = self.send_goal_and_wait(GOAL_JOINTS, allow_surprise=True)
        if self.surprise_sent:
            self.get_logger().info(
                'La escena cambio durante la ejecucion: eso es lo que dispara el '
                'replan del global planner.')
        else:
            self.get_logger().warning(
                'La escena NO cambio: no llego a publicarse ninguna solucion '
                'global.')
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
