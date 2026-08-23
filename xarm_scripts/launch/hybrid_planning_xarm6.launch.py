#!/usr/bin/env python3
"""
Hybrid Planning para el xArm6 - Clase 5 (Grippers y Planeamiento Hibrido).

Levanta los tres componentes de la arquitectura de hybrid planning de MoveIt 2
dentro de un mismo contenedor de nodos componibles:

    HybridPlanningManager  el "cerebro": recibe la accion HybridPlanner y decide
                           cuando invocar al global y al local planner.
    GlobalPlannerComponent resuelve el problema completo con OMPL, sin
                           restriccion de tiempo real, y publica la trayectoria.
    LocalPlannerComponent  recorre esa trayectoria a 50 Hz, manda comandos al
                           controlador y avisa si deja de ser valida.

Se lanzan como componentes en un container multihilo porque el local planner
tiene que correr a frecuencia fija: cada uno en su proceso agregaria latencia de
IPC en el lazo de control.

Prerequisito - la simulacion tiene que estar corriendo en otra terminal:

  ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \\
      world:=table_gz.world add_gripper:=true

Y despues:

  ros2 launch xarm_scripts hybrid_planning_xarm6.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml

from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    """Lee un yaml del share de un paquete y lo devuelve como dict."""
    absolute_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(absolute_path, 'r') as handle:
        return yaml.safe_load(handle)


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    run_demo = LaunchConfiguration('run_demo').perform(context) in ('True', 'true')

    # Descripcion del robot, SRDF, cinematica y pipeline de OMPL. Se arma con el
    # mismo builder que usa el resto del repo, en vez de apuntar a rutas fijas.
    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            controllers_name='fake_controllers',
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            add_gripper=add_gripper,
        )
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    robot_description = moveit_config.robot_description
    robot_description_semantic = moveit_config.robot_description_semantic
    kinematics = moveit_config.robot_description_kinematics
    joint_limits = moveit_config.joint_limits

    # El global planner es un moveit_cpp, y espera la config del pipeline con la
    # forma {"ompl": {...}}: los parametros generales del pipeline MAS la seccion
    # del grupo. Se cargan los dos yaml explicitamente; si solo se pasa la parte
    # general, OMPL avisa "Cannot find planning configuration for group 'xarm6'"
    # y cae a sus defaults, ignorando el default_planner_config del grupo.
    xarm_type = '{}{}'.format(
        robot_type.perform(context),
        dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    ompl_config = load_yaml(
        'xarm_moveit_config', 'config/moveit_configs/ompl_planning.yaml')
    ompl_config.update(
        load_yaml('xarm_moveit_config',
                  'config/{}/ompl_planning.yaml'.format(xarm_type)))
    planning_pipelines_config = {'ompl': ompl_config}

    common_params = load_yaml(
        'xarm_scripts', 'config/hybrid_planning/common_hybrid_planning_params.yaml')
    global_params = load_yaml(
        'xarm_scripts', 'config/hybrid_planning/global_planner.yaml')
    local_params = load_yaml(
        'xarm_scripts', 'config/hybrid_planning/local_planner.yaml')
    manager_params = load_yaml(
        'xarm_scripts', 'config/hybrid_planning/hybrid_planning_manager.yaml')

    use_sim_time = {'use_sim_time': True}

    container = ComposableNodeContainer(
        name='hybrid_planning_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_hybrid_planning',
                plugin='moveit::hybrid_planning::GlobalPlannerComponent',
                name='global_planner',
                parameters=[
                    common_params,
                    global_params,
                    robot_description,
                    robot_description_semantic,
                    kinematics,
                    joint_limits,
                    planning_pipelines_config,
                    use_sim_time,
                ],
            ),
            ComposableNode(
                package='moveit_hybrid_planning',
                plugin='moveit::hybrid_planning::LocalPlannerComponent',
                name='local_planner',
                parameters=[
                    common_params,
                    local_params,
                    robot_description,
                    robot_description_semantic,
                    kinematics,
                    use_sim_time,
                ],
            ),
            ComposableNode(
                package='moveit_hybrid_planning',
                plugin='moveit::hybrid_planning::HybridPlanningManager',
                name='hybrid_planning_manager',
                parameters=[
                    common_params,
                    manager_params,
                    use_sim_time,
                ],
            ),
        ],
        output='screen',
    )

    # El local planner necesita un JointGroupPositionController, no el
    # JointTrajectoryController de la simulacion. Se spawnea con param-file propio
    # para no tocar xarm_controller/config/, y se desactiva el xarm6_traj_controller
    # porque los dos reclaman las mismas command interfaces de posicion y
    # ros2_control no permite dos controladores activos sobre la misma interfaz.
    position_controller_params = os.path.join(
        get_package_share_directory('xarm_scripts'),
        'config', 'hybrid_planning', 'xarm6_position_controller.yaml')

    switch_to_position_control = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'ros2 control set_controller_state xarm6_traj_controller inactive '
            '  --controller-manager /controller_manager || true; '
            'ros2 run controller_manager spawner '
            '  xarm6_joint_group_position_controller '
            '  --controller-manager /controller_manager '
            '  --controller-type position_controllers/JointGroupPositionController '
            '  --param-file ' + position_controller_params,
        ],
        output='screen',
    )

    actions = [container, TimerAction(period=4.0, actions=[switch_to_position_control])]

    if run_demo:
        # El demo espera a que el contenedor levante los tres componentes antes
        # de mandar la meta.
        demo_node = Node(
            package='xarm_scripts',
            executable='hybrid_planning_demo',
            name='hybrid_planning_demo',
            output='screen',
            parameters=[
                common_params,
                robot_description,
                robot_description_semantic,
                use_sim_time,
            ],
        )
        actions.append(TimerAction(period=12.0, actions=[demo_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('robot_type', default_value='xarm'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('add_gripper', default_value='true'),
        DeclareLaunchArgument(
            'run_demo', default_value='true',
            description='false = solo levanta el planner, sin mandar ninguna meta'),
        OpaqueFunction(function=launch_setup),
    ])
