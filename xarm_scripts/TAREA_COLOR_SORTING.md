# Tarea: Clasificación de Colores con Robot

## Objetivo
Crear un sistema completo de clasificación de cubos de colores que detecte y coloque todos los cubos en el bote en el orden **R (Rojo), G (Verde), B (Azul)**. Debes crear el script `color_sorter.py` que controle el robot. El archivo launch (`color_sorting.launch.py`) ya está proporcionado.

## Preparaciones

Instala las dependencias necesarias:

```bash
sudo apt-get install ros-humble-tf-transformations
```

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

## Entregables

1. **Código completo:**
   - `color_sorter.py` funcional con ambos modos (target_color y auto_mode)

2. **Evidencia de funcionamiento:**
   - Video o captura de pantalla mostrando el sistema funcionando
   - El video debe mostrar la secuencia completa: R → G → B

## Calificación

- **`target_color`**: 14 puntos
- **`auto_mode`**: 6 puntos
- **Total**: 20 puntos