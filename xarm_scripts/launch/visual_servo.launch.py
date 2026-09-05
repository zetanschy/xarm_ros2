#!/usr/bin/env python3
"""Visual servoing con el xArm6 - Tarea 2 (Clase 5: Grippers y Planeamiento Hibrido).

Deja la escena lista para que el alumno solo tenga que escribir su nodo de
control. Hace tres cosas, en este orden:

  1. Lleva el brazo a una pose de observacion desde la que la camara de muneca ve
     el riel completo. Se manda como trayectoria al xarm6_traj_controller, que en
     ese momento todavia esta activo.
  2. Cambia de controlador: apaga el xarm6_traj_controller y prende el
     xarm6_joint_group_position_controller. Los dos reclaman las mismas command
     interfaces de posicion, y ros2_control no permite dos controladores activos
     sobre la misma interfaz. Es el mismo cambio que hace el hybrid planning de
     la Clase 5, y reusa su mismo archivo de parametros.
  3. Levanta el servo_node y el nodo que mueve el marcador.

OJO: despues del paso 2, MoveIt ya NO puede ejecutar trayectorias (su controlador
esta inactivo). Todo el movimiento del brazo pasa a ser del servo.

Prerequisito - la simulacion tiene que estar corriendo en otra terminal:

  ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \\
      world:=table_aruco.world add_gripper:=true

Y despues:

  ros2 launch xarm_scripts visual_servo.launch.py            # marcador lento
  ros2 launch xarm_scripts visual_servo.launch.py mode:=fast # marcador rapido
  ros2 launch xarm_scripts visual_servo.launch.py mode:=static
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

# Pose de observacion, en valores articulares: link6 en (-0.30, -0.40, 0.437) de
# link_base con el gripper hacia abajo, o sea el TCP en 0.265. La camara queda
# unos 0.18 m sobre el marcador, y desde ahi el rectangulo entero entra en cuadro
# (verificado: desde las cuatro esquinas el marcador se sigue detectando, a 68 px).
#
# La altura importa. Con el robot movil el marcador viaja mucho mas alto que antes
# (cubierta en 1.108 contra la mesa en 1.015), asi que la pose de observacion
# vieja dejaba la camara a solo 9 cm del marcador: cubria menos mesa que el propio
# recorrido y el marcador se salia de cuadro.
OBSERVATION_JOINTS = [-2.2148, 0.0672, -1.3772, 0.0122, 1.3084, 0.9311]


def load_yaml(package_name, file_path):
    absolute_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(absolute_path, 'r') as handle:
        return yaml.safe_load(handle)


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    mode = LaunchConfiguration('mode').perform(context)
    move_target = LaunchConfiguration('move_target').perform(context) in ('True', 'true')
    user_node = LaunchConfiguration('node').perform(context).strip()

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            controllers_name='fake_controllers',
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            add_gripper=add_gripper,
            add_wrist_camera='true',
        )
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    servo_params = {'moveit_servo': load_yaml(
        'xarm_scripts', 'config/visual_servo/xarm6_servo.yaml')}
    use_sim_time = {'use_sim_time': True}

    # 0) Reset de la escena. Sin esto, relanzar despues de un agarre exitoso
    # deja el cubo colgando del gripper: al mover el brazo se cae en cualquier
    # lado de la mesa y ya no vuelve a la bandeja nunca.
    #
    # El orden importa. Primero se abre el gripper (si no, el cubo se
    # teletransporta y despues cae de los dedos). Despues se manda el carro al
    # robot movil al centro y se espera a que llegue: si se teletransporta el
    # cubo mientras el robot esta en una esquina, el cubo aparece al lado del
    # robot en vez de encima, se queda en la mesa, y ya no viaja con el.
    #
    # OJO al verificarlo a mano: `ign model --model aruco_cube --pose` devuelve
    # una pose cacheada y miente. La pose viva sale de
    # `ign topic -e -t /world/default/dynamic_pose/info`.
    reset_scene = ExecuteProcess(
        cmd=[
            'bash', '-c',
            # Antes que nada, esperar a que la simulacion de la Terminal 1 tenga
            # los controladores cargados. Gazebo tarda entre 60 y 90 s en llegar
            # ahi, y lanzar esta terminal antes es lo que va a pasar siempre.
            #
            # Sin esta espera el launch seguia igual: publicaba la trayectoria a
            # la pose de observacion sin que nadie la ejecutara, cambiaba de
            # controlador, levantaba el servo, y dejaba el brazo en la pose de
            # spawn -- SIN UN SOLO MENSAJE DE ERROR. El sintoma para el alumno es
            # "no detecta el marcador", que manda a buscar el problema al lugar
            # equivocado.
            #
            # Se espera a que el controlador EXISTA, no a que este activo: si
            # este launch ya corrio, quedo inactivo, y reactivarlo es justo lo
            # que hace el paso siguiente.
            'CM=/controller_manager; '
            'echo "esperando los controladores de la simulacion..."; '
            'for i in $(seq 1 90); do '
            '  if ros2 control list_controllers --controller-manager $CM 2>/dev/null '
            '       | sed \'s/\\x1b\\[[0-9;]*m//g\' | grep -q "^xarm6_traj_controller"; then '
            '    LISTO=1; break; '
            '  fi; '
            '  sleep 2; '
            'done; '
            'if [ -z "$LISTO" ]; then '
            '  echo; '
            '  echo "=========================================================="; '
            '  echo "ERROR: la simulacion no aparecio en 180 s."; '
            '  echo "Arranca primero la Terminal 1 y espera a ver"; '
            '  echo "  Configured and activated xarm6_traj_controller"; '
            '  echo "antes de lanzar esta."; '
            '  echo "=========================================================="; '
            '  exit 1; '
            'fi; '
            'echo "controladores listos"; '
            "ros2 topic pub --once /xarm_gripper_traj_controller/joint_trajectory "
            "trajectory_msgs/msg/JointTrajectory "
            "'{joint_names: [drive_joint], points: [{positions: [0.0], "
            "time_from_start: {sec: 1, nanosec: 0}}]}' > /dev/null 2>&1 || true; "
            'ros2 topic pub -r 20 -t 40 /mobile_robot/cmd_x '
            '  std_msgs/msg/Float64 "{data: 0.0}" > /dev/null 2>&1 || true; '
            'ros2 topic pub -r 20 -t 40 /mobile_robot/cmd_y '
            '  std_msgs/msg/Float64 "{data: 0.0}" > /dev/null 2>&1 || true; '
            'sleep 2; '
            'ign service -s /world/default/set_pose '
            '  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean '
            '  --timeout 3000 '
            # Sin `id`: Gazebo lo asigna al cargar el mundo y cambia entre
            # arranques (se vieron 28 y 36). Con un id equivocado la peticion
            # falla en silencio aunque lleve el nombre correcto, y el reset no
            # hace nada. Y sin comas: el formato de texto de protobuf no las usa
            # como separador, y con ellas se pierden campos.
            '  --req \'name: "aruco_cube" '
            '     position { x: 0.2 y: -0.8 z: 1.096 } '
            "     orientation { x: 0 y: 0 z: 0 w: 1 }' > /dev/null 2>&1 || true; "
            'sleep 2; '
            'echo "escena reseteada"; '
            'exit 0',
        ],
        output='screen',
    )

    # 1) A la pose de observacion. Antes hay que asegurarse de que el
    # xarm6_traj_controller este ACTIVO: si este launch ya corrio una vez contra
    # el mismo Gazebo, quedo inactivo del paso 2, y entonces la trayectoria se
    # publica, nadie la ejecuta, y el brazo se queda donde lo dejo el intento
    # anterior -- normalmente abajo, con el marcador fuera de cuadro. Relanzar
    # sin reiniciar Gazebo tiene que funcionar: en clase se relanza mucho.
    joints = ', '.join(str(v) for v in OBSERVATION_JOINTS)
    goto_observation = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'CM=/controller_manager; '
            'ros2 control set_controller_state xarm6_joint_group_position_controller '
            '  inactive --controller-manager $CM > /dev/null 2>&1 || true; '
            'ros2 control set_controller_state xarm6_traj_controller active '
            '  --controller-manager $CM > /dev/null 2>&1 || true; '
            'for i in $(seq 1 20); do '
            '  ros2 action list 2>/dev/null '
            '    | grep -q "^/xarm6_traj_controller/follow_joint_trajectory$" && break; '
            '  sleep 1; '
            'done; '
            # Se manda por la ACCION y no por el topico
            # /xarm6_traj_controller/joint_trajectory: la accion bloquea hasta
            # que el brazo llega y dice si el goal fue aceptado o rechazado. Con
            # el topico no hay ninguna respuesta -- si el controlador esta
            # inactivo la trayectoria se publica al vacio, el launch sigue
            # contento, y el alumno termina con el brazo en la pose de spawn
            # buscando el problema en su codigo de vision.
            # `ros2 action send_goal` se cuelga PARA SIEMPRE si el servidor de
            # accion todavia no existe (el controlador recien activado tarda un
            # instante en publicarlo). Sin el timeout, el launch se queda mudo
            # despues de "escena reseteada" y no pasa nada mas.
            'OUT=$(timeout 40 ros2 action send_goal '
            '  /xarm6_traj_controller/follow_joint_trajectory '
            '  control_msgs/action/FollowJointTrajectory '
            "  '{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], "
            'points: [{positions: [' + joints + '], '
            "time_from_start: {sec: 4, nanosec: 0}}]}}' 2>&1); "
            'echo "$OUT" | tail -2; '
            'if ! echo "$OUT" | grep -q SUCCEEDED; then '
            '  echo; '
            '  echo "=========================================================="; '
            '  echo "ERROR: el brazo no llego a la pose de observacion."; '
            '  echo "Desde donde quedo, la camara no ve el marcador y nada de"; '
            '  echo "lo que sigue va a funcionar. Reinicia la Terminal 1 y"; '
            '  echo "volve a lanzar esta."; '
            '  echo "=========================================================="; '
            '  exit 1; '
            'fi; '
            'echo "en la pose de observacion"',
        ],
        output='screen',
    )

    # 2) Cambio de controlador. Idempotente: si el launch se relanza sin
    # reiniciar Gazebo, el controlador de posicion ya existe y solo hay que
    # reactivarlo, asi que el spawner fallaria con exit 1.
    position_controller_params = os.path.join(
        get_package_share_directory('xarm_scripts'),
        'config', 'hybrid_planning', 'xarm6_position_controller.yaml')

    switch_to_position_control = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'CM=/controller_manager; '
            'ros2 control set_controller_state xarm6_traj_controller inactive '
            '  --controller-manager $CM > /dev/null 2>&1 || true; '
            'if ros2 control list_controllers --controller-manager $CM 2>/dev/null '
            '     | grep -q xarm6_joint_group_position_controller; then '
            '  echo "controlador de posicion ya cargado; se reactiva"; '
            '  ros2 control set_controller_state '
            '    xarm6_joint_group_position_controller active '
            '    --controller-manager $CM > /dev/null 2>&1 || true; '
            'else '
            '  ros2 run controller_manager spawner '
            '    xarm6_joint_group_position_controller '
            '    --controller-manager $CM '
            '    --controller-type position_controllers/JointGroupPositionController '
            '    --param-file ' + position_controller_params + '; '
            'fi; '
            'exit 0',
        ],
        output='screen',
    )

    # 3) El servo. Arranca DETENIDO: hay que llamar a /servo_node/start_servo
    # para que empiece a mandar comandos. Es a proposito - un servo que arranca
    # solo es un brazo que se mueve sin que nadie se lo haya pedido.
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    # Los pasos se encadenan por evento, NO por temporizador. Cada
    # `ros2 control set_controller_state` tarda unos 4 s solo en descubrir el
    # servicio del controller_manager, asi que con timers fijos el cambio de
    # controlador le caia encima al movimiento a la pose de observacion y lo
    # cortaba a medias: el brazo quedaba a mitad de camino y el marcador fuera
    # de cuadro. Con OnProcessExit no importa cuanto tarde cada paso.
    after_goto = [switch_to_position_control]
    after_switch = [servo_node]

    if move_target:
        after_switch.append(Node(
            package='xarm_scripts',
            executable='aruco_target_mover',
            name='aruco_target_mover',
            output='screen',
            parameters=[{'mode': mode}, use_sim_time],
        ))

    # El nodo del alumno va ULTIMO, y solo si se pidio con node:=<ejecutable>.
    # Sirve para no tener que abrir una tercera terminal. Tiene que arrancar
    # despues del servo: su llamada a /servo_node/start_servo espera 15 s, que
    # alcanza para que el servo termine de levantar, pero al reves no funciona.
    if user_node:
        after_switch.append(Node(
            package='xarm_scripts',
            executable=user_node,
            name=user_node,
            output='screen',
            parameters=[use_sim_time],
        ))

    def continue_if_ok(next_actions):
        """Sigue con el paso siguiente solo si el anterior salio bien.

        Si un paso falla, lo que corresponde es parar todo: seguir levantando el
        servo sobre un brazo que quedo en la pose equivocada solo sirve para que
        el error aparezca 20 minutos despues, disfrazado de "no detecta el
        marcador".
        """
        def handler(event, context):
            if event.returncode != 0:
                return [LogInfo(msg='paso fallido: se aborta el launch'), Shutdown()]
            return next_actions
        return handler

    return [
        reset_scene,
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=reset_scene, on_exit=continue_if_ok([goto_observation]))),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=goto_observation, on_exit=continue_if_ok(after_goto))),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=switch_to_position_control, on_exit=continue_if_ok(after_switch))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('robot_type', default_value='xarm'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('add_gripper', default_value='true'),
        DeclareLaunchArgument(
            'mode', default_value='slow',
            description='velocidad del marcador: static | slow | medium | fast'),
        DeclareLaunchArgument(
            'move_target', default_value='true',
            description='false = no lanza el nodo que mueve el robot movil'),
        DeclareLaunchArgument(
            'node', default_value='',
            description='ejecutable de xarm_scripts a lanzar cuando el servo este '
                        'listo, p.ej. node:=aruco_servo. Vacio = no lanza nada, y '
                        'el nodo se corre a mano en otra terminal.'),
        OpaqueFunction(function=launch_setup),
    ])
