# xarm_scripts - Guía de Uso de MoveIt 2 Python API
ROBOT_STATE
Este directorio contiene ejemplos y scripts para controlar robots xArm usando la API de Python de MoveIt 2.
## Tabla de Contenidos

1. [Bloque Inicial de Configuración](#bloque-inicial-de-configuración)
2. [Scripts de MoveIt Python API](#scripts-de-moveit-python-api)
3. [Script pymoveit2](#script-pymoveit2)
4. [Launch de MoveIt Servo](#launch-de-moveit-servo)

---

## Bloque Inicial de Configuración

Todos los scripts de MoveIt Python API en este directorio comienzan con un bloque de configuración similar. Esta sección explica por qué es necesario y qué hace cada parte.

### 1. Parámetro `use_sim_time`

```python
if node.has_parameter('use_sim_time'):
    use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
else:
    node.declare_parameter('use_sim_time', False)
    use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
```

**Razón:** 
- Cuando se ejecuta con simulación (Gazebo), ROS 2 usa el tiempo de simulación (`/clock`) en lugar del tiempo del sistema.
- Este parámetro debe ser `true` para simulación y `false` para hardware real.
- MoveItPy necesita conocer este parámetro para sincronizar correctamente con el tiempo de simulación o real.

**Uso:**
```bash
# Para simulación
ros2 run xarm_scripts example_robot_trajectory --ros-args -p use_sim_time:=true

# Para hardware real
ros2 run xarm_scripts example_robot_trajectory --ros-args -p use_sim_time:=false
```

### 2. MoveItConfigsBuilder

```python
moveit_config_builder = MoveItConfigsBuilder(
    context=None,  # No launch context needed for standalone script
    controllers_name='fake_controllers',  # Use fake controllers for testing
    dof=6,  # xarm6 has 6 DOF
    robot_type='xarm',
    prefix='',
    limited=True,
)
moveit_config_builder.moveit_cpp(
    file_path=get_package_share_directory("xarm_moveit_config") + "/config/moveit_planning_python.yaml"
)
moveit_config_dict = moveit_config_builder.to_dict()
```

**Razón:**
- `MoveItConfigsBuilder` es una clase personalizada de `uf_ros_lib` diseñada específicamente para robots xArm.
- Automáticamente resuelve las rutas de URDF/SRDF y carga todas las configuraciones necesarias de MoveIt.
- `moveit_cpp()` carga el archivo de configuración de planificación que especifica los pipelines de planificación disponibles (OMPL, Pilz, CHOMP, etc.).
- `to_dict()` convierte todas las configuraciones en un diccionario que MoveItPy puede usar.

**Parámetros importantes:**
- `controllers_name`: `'fake_controllers'` para simulación/testing, `'controllers'` para hardware real
- `dof`: Grados de libertad del robot (5, 6, o 7)
- `robot_type`: Tipo de robot (`'xarm'`, `'lite'`, etc.)
- `limited`: Si el robot tiene límites de articulación reducidos

### 3. Agregar `use_sim_time` al Config Dict

```python
if 'use_sim_time' not in moveit_config_dict:
    moveit_config_dict['use_sim_time'] = use_sim_time
```

**Razón:**
- MoveItPy crea su propio nodo interno que también necesita conocer `use_sim_time`.
- Este parámetro debe estar en el diccionario de configuración antes de inicializar MoveItPy.

### 4. QoS Override Fix

```python
MoveItConfigsBuilder._add_qos_overrides_for_sim_time(moveit_config_dict)
```

**Razón:**
- Cuando `use_sim_time=True`, MoveItPy necesita suscribirse al tópico `/clock` con configuraciones QoS específicas.
- Sin estos overrides, MoveItPy intenta configurar QoS después de crear el nodo, lo cual falla.
- Esta función agrega los overrides necesarios al diccionario de configuración **antes** de la inicialización:
  - `durability: transient_local`
  - `reliability: reliable`
  - `history: keep_last`
  - `depth: 10`

**Sin este fix:** MoveItPy no puede sincronizar correctamente con el tiempo de simulación y puede fallar al recibir estados del robot.

---

## Scripts de MoveIt Python API

### Scripts Disponibles

1. **`moveit_py_joint_goal.py`** - Ejemplo básico de movimiento a configuración de articulaciones
2. **`moveit_py_pose_goal.py`** - Ejemplo básico de movimiento a pose objetivo
3. **`example_robot_state.py`** - Manipulación y consulta del estado del robot
4. **`example_robot_trajectory.py`** - Inspección y manipulación de trayectorias
5. **`example_collision.py`** - Ejemplo de planificación con objetos de colisión
6. **`example_kinematic_constraints.py`** - Uso de restricciones cinemáticas
7. **`example_advanced_planning.py`** - Planificación avanzada con múltiples pipelines
8. **`example_multi_pipeline.py`** - Ejemplo de planificación multi-pipeline

### API de MoveIt Python - Conceptos Clave

#### 1. Inicialización

```python
from moveit.planning import MoveItPy
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

# Construir configuración
moveit_config_builder = MoveItConfigsBuilder(...)
moveit_config_dict = moveit_config_builder.to_dict()

# Inicializar MoveItPy
xarm_moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
planning_component = xarm_moveit.get_planning_component("xarm6")
```

#### 2. Establecer Estado Inicial

```python
# Usar el estado actual del robot
planning_component.set_start_state_to_current_state()

# O establecer un estado personalizado
robot_state = RobotState(robot_model)
robot_state.joint_positions = {"joint1": 0.5, "joint2": 0.0, ...}
planning_component.set_start_state(robot_state)
```

#### 3. Establecer Estado Objetivo

**Opción 1: Pose Goal**
```python
pose_goal = PoseStamped()
pose_goal.header.frame_id = "link_base"
pose_goal.pose.position.x = 0.3
pose_goal.pose.position.y = 0.2
pose_goal.pose.position.z = 0.2
pose_goal.pose.orientation.w = 1.0
planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
```

**Opción 2: Joint Goal**
```python
from moveit.core.kinematic_constraints import construct_joint_constraint

robot_state = RobotState(robot_model)
robot_state.joint_positions = {"joint1": 0.5, "joint2": 0.0, ...}
joint_constraint = construct_joint_constraint(
    robot_state=robot_state,
    joint_model_group=joint_model_group,
)
planning_component.set_goal_state(motion_plan_constraints=[joint_constraint])
```

**Opción 3: Named Configuration (desde SRDF)**
```python
planning_component.set_goal_state(configuration_name="ready")
```

#### 4. Planificar

```python
# Planificación simple
plan_result = planning_component.plan()

# Planificación con parámetros personalizados
from moveit.planning import PlanRequestParameters
plan_parameters = PlanRequestParameters(xarm_moveit)
plan_parameters.planning_pipeline = "ompl"
plan_parameters.planner_id = "RRTConnectkConfigDefault"
plan_parameters.max_velocity_scaling_factor = 0.5
plan_result = planning_component.plan(plan_parameters)
```

#### 5. Ejecutar

```python
if plan_result:
    robot_trajectory = plan_result.trajectory
    xarm_moveit.execute(planning_group, robot_trajectory, blocking=True)
```

#### 6. Trabajar con Planning Scene

```python
planning_scene_monitor = xarm_moveit.get_planning_scene_monitor()

# Leer estado actual
with planning_scene_monitor.read_only() as scene:
    current_state = scene.current_state
    planning_frame = scene.planning_frame

# Agregar objetos de colisión
with planning_scene_monitor.read_write() as scene:
    scene.apply_collision_object(collision_object)
    scene.current_state.update()
```

#### 7. Trabajar con RobotTrajectory

Según la [documentación oficial](https://moveit.picknik.ai/main/doc/api/python_api/_autosummary/moveit.core.robot_trajectory.html):

```python
robot_trajectory = plan_result.trajectory

# Propiedades disponibles
duration = robot_trajectory.duration  # Duración total
avg_duration = robot_trajectory.average_segment_duration  # Duración promedio de segmentos

# Obtener mensaje ROS
trajectory_msg = robot_trajectory.get_robot_trajectory_msg()
num_waypoints = len(trajectory_msg.joint_trajectory.points)

# Obtener duraciones entre waypoints
durations = robot_trajectory.get_waypoint_durations()

# Aplicar suavizado
robot_trajectory.apply_ruckig_smoothing(
    velocity_scaling_factor=0.5,
    acceleration_scaling_factor=0.5
)
```

### Ejecutar Scripts

**Con launch file unificado:**
```bash
# Ejecutar cualquier script usando el launch file unificado
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_robot_trajectory

# Con parámetros personalizados
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_joint_goal use_sim_time:=true

# Lista de scripts disponibles:
# - example_joint_goal
# - example_pose_goal
# - example_robot_state
# - example_robot_trajectory
# - example_collision
# - example_kinematic_constraints
# - example_multi_pipeline
# - example_planning_scene
# - example_transforms
# - example_advanced_planning
# - test_pymoveit2_api
```

**Directamente con ros2 run:**
```bash
# Simulación
ros2 run xarm_scripts example_robot_trajectory --ros-args -p use_sim_time:=true

# Hardware real
ros2 run xarm_scripts example_robot_trajectory --ros-args -p use_sim_time:=false
```

---

## Script pymoveit2

El script `test_pymoveit2_api.py` usa la biblioteca `pymoveit2`, que es una interfaz de alto nivel diferente a MoveItPy.

### Diferencias con MoveItPy

- **pymoveit2**: Interfaz simplificada, más fácil de usar, menos control
- **MoveItPy**: Interfaz completa, más control, acceso a todas las funcionalidades

### Uso de pymoveit2

```python
from pymoveit2 import MoveIt2

moveit2 = MoveIt2(
    node=node,
    joint_names=["joint1", "joint2", ...],
    base_link_name="link_base",
    end_effector_name="link_eef",
    group_name="xarm6",
    callback_group=callback_group,
)

# Mover a configuración de articulaciones
moveit2.move_to_configuration([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
moveit2.wait_until_executed()
```

### Ejecutar test_pymoveit2_api

```bash
# Con parámetros personalizados
ros2 run xarm_scripts test_pymoveit2_api --ros-args \
    -p joint_positions:="[0.5, 0.0, 0.0, 0.0, 0.0, 0.0]" \
    -p use_sim_time:=true

# Con ejecución síncrona
ros2 run xarm_scripts test_pymoveit2_api --ros-args \
    -p joint_positions:="[0.5, 0.0, 0.0, 0.0, 0.0, 0.0]" \
    -p synchronous:=True

# Con cancelación después de tiempo
ros2 run xarm_scripts test_pymoveit2_api --ros-args \
    -p joint_positions:="[0.5, 0.0, 0.0, 0.0, 0.0, 0.0]" \
    -p synchronous:=False \
    -p cancel_after_secs:=2.0
```

**Parámetros disponibles:**
- `joint_positions`: Array de posiciones de articulaciones en radianes
- `synchronous`: Si esperar hasta que la ejecución termine (default: False)
- `cancel_after_secs`: Cancelar ejecución después de N segundos (solo si synchronous=False)
- `planner_id`: ID del planificador a usar (default: "RRTConnectkConfigDefault")

---

## Launch de MoveIt Servo

### xarm_moveit_servo_fake.launch.py

Este launch file inicia MoveIt Servo en modo "fake" (simulación/testing).

**¿Qué es MoveIt Servo?**
- Permite control teleoperado del robot en tiempo real
- Responde a comandos de velocidad/posición incremental
- Útil para control con joystick, teclado, o interfaces de usuario

**Ejecutar:**

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py
```

**Parámetros disponibles:**
- `dof`: Grados de libertad (default: 6)
- `prefix`: Prefijo para nombres de articulaciones (default: '')
- `hw_ns`: Namespace de hardware (default: 'xarm')
- `limited`: Si el robot tiene límites reducidos (default: True)
- `robot_type`: Tipo de robot (default: 'xarm')
- `add_gripper`: Agregar gripper (default: False)
- `add_vacuum_gripper`: Agregar vacuum gripper (default: False)

**Ejemplo con parámetros:**
```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py \
    dof:=7 \
    robot_type:=xarm \
    add_gripper:=true
```

**Diferencia entre `_fake` y `_realmove`:**
- `_fake`: Usa controladores simulados, para testing/simulación
- `_realmove`: Usa controladores reales, para hardware físico

---

## Resumen de Comandos

### Launch File Unificado

**Usar el launch file unificado (recomendado):**
```bash
# Ejemplos básicos
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_joint_goal
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_pose_goal use_sim_time:=true

# Ejemplos avanzados
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_robot_state use_sim_time:=true
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_robot_trajectory use_sim_time:=true
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_collision use_sim_time:=true
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_kinematic_constraints use_sim_time:=true
ros2 launch xarm_scripts xarm_scripts.launch.py script:=example_multi_pipeline use_sim_time:=true
```

### Scripts MoveIt Python API (ros2 run directo)

```bash
# Ejemplos básicos
ros2 run xarm_scripts example_joint_goal --ros-args -p use_sim_time:=true
ros2 run xarm_scripts example_pose_goal --ros-args -p use_sim_time:=true

# Ejemplos avanzados
ros2 run xarm_scripts example_robot_state --ros-args -p use_sim_time:=true
ros2 run xarm_scripts example_robot_trajectory --ros-args -p use_sim_time:=true
ros2 run xarm_scripts example_collision --ros-args -p use_sim_time:=true
ros2 run xarm_scripts example_kinematic_constraints --ros-args -p use_sim_time:=true
```

### Script pymoveit2

```bash
# Con launch file
ros2 launch xarm_scripts xarm_scripts.launch.py script:=test_pymoveit2_api use_sim_time:=true

# O directamente
ros2 run xarm_scripts test_pymoveit2_api --ros-args \
    -p joint_positions:="[0.5, 0.0, 0.0, 0.0, 0.0, 0.0]" \
    -p use_sim_time:=true
```

### MoveIt Servo

```bash
ros2 launch xarm_moveit_servo xarm_moveit_servo_fake.launch.py
```

---

## Referencias

- [MoveIt 2 Python API Documentation](https://moveit.picknik.ai/main/doc/api/python_api/)
- [MoveIt 2 Python API Announcement](https://moveit.ai/moveit/ros/python/google/2023/02/15/MoveIt-Humble-Release.html)
- [RobotTrajectory API](https://moveit.picknik.ai/main/doc/api/python_api/_autosummary/moveit.core.robot_trajectory.html)
- [Kinematic Constraints API](https://moveit.picknik.ai/main/doc/api/python_api/_autosummary/moveit.core.kinematic_constraints.html)

