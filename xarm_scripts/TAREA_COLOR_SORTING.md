# Tarea: Clasificación de Colores con Robot

## Objetivo
Crear un sistema completo de clasificación de cubos de colores que detecte y coloque todos los cubos en el bote en el orden **R (Rojo), G (Verde), B (Azul)**. El sistema debe consistir en:
1. Un script `color_sorter.py` que controle el robot
2. Un archivo launch que ejecute simultáneamente `color_detector` y `color_sorter`

## Comandos para Ejecutar

### Paso 1: Iniciar Gazebo con MoveIt

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py robot_x:=0.0 robot_y:=-0.4 robot_z:=0.45 robot_yaw:=1.5708 world:=table_color_objects.world add_gripper:=true
```

### Paso 2: Ejecutar el sistema de clasificación

```bash
ros2 launch xarm_scripts color_sorting.launch.py
```

## Descripción de la Tarea

### Paso 1: Implementar `target_color` (Modo Individual)

**Primero debes implementar la funcionalidad para recoger un color específico mediante parámetro.**

Tu script `color_sorter.py` debe:
- Recibir un parámetro `target_color` (R, G o B)
- Suscribirse al tópico `/color_coordinates` para recibir las coordenadas de los cubos detectados
- Cuando se detecte el color especificado, ejecutar la rutina de pick and place
- Colocar el cubo en el bote

**Pista - Posición del bote:**
```python
CAN_X = -0.2
CAN_Y = -0.58
CAN_Z = 0.3  # Altura de colocación
```
Estas coordenadas están en el marco de referencia `link_base`

**Secuencia de movimiento:**
1. Abrir gripper
2. Mover a altura segura
3. Acercarse a la posición del cubo (approach)
4. Bajar a la posición exacta del cubo
5. Cerrar gripper
6. Elevar el cubo
7. Mover hacia el bote
8. Bajar a la posición de colocación
9. Abrir gripper (soltar)
10. Retraerse

**Prueba individualmente cada color:**
```bash
# Probar con rojo
ros2 launch xarm_scripts color_sorting.launch.py target_color:=R

# Probar con verde
ros2 launch xarm_scripts color_sorting.launch.py target_color:=G

# Probar con azul
ros2 launch xarm_scripts color_sorting.launch.py target_color:=B
```

### Paso 2: Implementar `auto_mode` (Secuencia Completa)

**Una vez que el modo `target_color` funcione correctamente, implementa el modo automático.**

El modo `auto_mode` debe:
- Detectar todos los colores disponibles
- Colocarlos en el bote en el orden **R → G → B**
- Esperar a que se detecten los tres colores antes de comenzar
- Ejecutar la secuencia completa automáticamente

**Ejecutar modo automático:**
```bash
ros2 launch xarm_scripts color_sorting.launch.py auto_mode:=true
```

### Crear el Launch File

Debes crear un archivo launch (`color_sorting.launch.py`) que:

1. **Ejecute `color_detector`** como un nodo
2. **Ejecute `color_sorter`** como otro nodo
3. Ambos nodos deben ejecutarse simultáneamente
4. Pasar los parámetros `target_color` y `auto_mode` al nodo `color_sorter`

Ejemplo de estructura:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_color_arg = DeclareLaunchArgument(
        'target_color',
        default_value='',
        description='Target color to pick (R/G/B). Empty for auto mode'
    )
    
    auto_mode_arg = DeclareLaunchArgument(
        'auto_mode',
        default_value='false',
        description='Auto mode: pick all colors in R-G-B order'
    )
    
    color_detector_node = Node(
        package='xarm_scripts',
        executable='color_detector',
        name='color_detector',
        output='screen'
    )
    
    color_sorter_node = Node(
        package='xarm_scripts',
        executable='color_sorter',
        name='color_sorter',
        output='screen',
        parameters=[{
            'target_color': LaunchConfiguration('target_color'),
            'auto_mode': LaunchConfiguration('auto_mode'),
        }]
    )
    
    return LaunchDescription([
        target_color_arg,
        auto_mode_arg,
        color_detector_node,
        color_sorter_node,
    ])
```

## Entregables

1. **Código completo:**
   - `color_sorter.py` funcional con ambos modos (target_color y auto_mode)
   - Archivo launch que ejecute ambos nodos

2. **Evidencia de funcionamiento:**
   - Video o captura de pantalla mostrando el sistema funcionando
   - El video debe mostrar la secuencia completa: R → G → B