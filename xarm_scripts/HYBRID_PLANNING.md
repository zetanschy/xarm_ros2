# Hybrid Planning con el xArm6 — Clase 5

Ejemplo de la arquitectura de *hybrid planning* de MoveIt 2 adaptada al xArm6.
Referencia: https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html

## Por qué existe

Con `move_group` (Clases 3 y 4) el ciclo es: planificar una vez → ejecutar. Si
el mundo cambia **después** de planificar, la trayectoria se ejecuta igual o se
aborta. Hybrid planning parte ese ciclo en dos lazos que corren a la vez:

| Componente | Qué hace | Ritmo |
|---|---|---|
| **Global Planner** | resuelve el problema completo con OMPL | lento, sin tiempo real |
| **Local Planner** | recorre esa trayectoria y comanda el robot | 50 Hz, tiempo real |
| **Hybrid Planning Manager** | el "cerebro": recibe la acción y decide cuándo invocar a cada uno | por eventos |

El local planner ve el mundo en cada iteración. Si la trayectoria global deja de
ser válida, avisa, y el manager decide qué hacer según su *planner logic plugin*.

## Los cuatro plugins

La arquitectura es toda plugins. Los que usa este ejemplo:

| Interfaz | Plugin usado | Dónde se configura |
|---|---|---|
| Planner logic | `moveit_hybrid_planning/ReplanInvalidatedTrajectory` | `config/hybrid_planning/hybrid_planning_manager.yaml` |
| Global planner | `moveit_hybrid_planning/MoveItPlanningPipeline` | `config/hybrid_planning/global_planner.yaml` |
| Trajectory operator | `moveit_hybrid_planning/SimpleSampler` | `config/hybrid_planning/local_planner.yaml` |
| Local constraint solver | `moveit_hybrid_planning/ForwardTrajectory` | `config/hybrid_planning/local_planner.yaml` |

El logic plugin es el interesante para la clase. Hay dos:

- `SinglePlanExecution`: planifica una vez y ejecuta. Si se invalida, aborta.
- `ReplanInvalidatedTrajectory`: cuando el local planner avisa, le pide al global
  planner un plan nuevo **sin parar la ejecución**. Es el que hace visible el
  comportamiento reactivo. Cambiar una línea del yaml y volver a correr es una
  buena demo de por sí.

## Por qué el demo de MoveIt funciona y el nuestro no funcionaba

Vale la pena mirarlo porque las diferencias no son cosméticas. Cuatro, en orden
de importancia:

### 1. La interfaz de control (la que más pesaba)

| | demo de MoveIt (Panda) | nuestro primer intento |
|---|---|---|
| controlador | `position_controllers/JointGroupPositionController` | `joint_trajectory_controller/JointTrajectoryController` |
| mensaje | `std_msgs/Float64MultiArray` | `trajectory_msgs/JointTrajectory` |
| frecuencia | 100 Hz | 50 Hz |

El local planner es un **lazo de control**: escribe una posición articular por
ciclo. Eso es exactamente lo que consume un `JointGroupPositionController`.

Un `JointTrajectoryController` espera *trayectorias* e interpola él mismo, con
tolerancias de path y de goal. Mandarle una trayectoria nueva de un solo punto 50
veces por segundo lo hace reiniciar la ejecución sin parar; la ejecución se
atrasa, viola tolerancias, y el brazo terminaba **clavado contra sus límites
articulares** y trabado contra la mesa — de ahí no lo sacaba ningún comando y
había que reiniciar Gazebo entre corridas.

El demo del Panda declara los dos controladores (`panda_arm_controller` y
`panda_joint_group_position_controller`) y el local planner sólo maneja el
segundo. Ahora hacemos lo mismo: `hybrid_planning_xarm6.launch.py` spawnea un
`xarm6_joint_group_position_controller` con su propio param-file y desactiva el
`xarm6_traj_controller` mientras corre el hybrid planning. No se toca
`xarm_controller/config/`.

### 2. Los obstáculos son placas finas, no cajas macizas

El demo de MoveIt usa `{0.5, 0.8, 0.01}` y `{1.0, 0.4, 0.01}`: **1 cm** de
espesor. Una placa invalida cualquier trayectoria que la cruce pero casi no le
quita espacio libre al brazo, así que siempre queda un camino alternativo. Con
cajas macizas el replan se queda sin solución.

### 3. Borran el obstáculo viejo al agregar los nuevos

```cpp
collision_object_1_.operation = collision_object_1_.REMOVE;   // sale la vieja
collision_object_2_.operation = collision_object_2_.ADD;      // entran dos nuevas
collision_object_3_.operation = collision_object_3_.ADD;
```

La escena **no acumula** obstáculos. Nuestro primer intento sólo agregaba.

### 4. El disparador es la publicación de la solución global

Se suscriben a `global_trajectory` y cambian la escena cuando el global planner
publica una solución: una vez por ciclo de planificación, y con operaciones
idempotentes, así que después del primer cambio la escena se queda quieta.
Nosotros disparábamos por progreso del movimiento.

Todo eso ya está portado. Lo que queda sin resolver es lo de la sección
siguiente, y es una limitación de los plugins de lógica de MoveIt, no de esta
adaptación.

## Correr

Terminal 1 — simulación:

```bash
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \
    world:=table_gz.world add_gripper:=true
```

Terminal 2 — los tres componentes más el demo:

```bash
ros2 launch xarm_scripts hybrid_planning_xarm6.launch.py
```

Sólo el planner, sin mandar ninguna meta (para mandarla a mano después):

```bash
ros2 launch xarm_scripts hybrid_planning_xarm6.launch.py run_demo:=false
```

## Lo que hace el demo

1. **Paso 0** — limpia la escena y manda el brazo a una pose conocida
   comandando el controlador **directo, sin planificar**. Si una corrida anterior
   dejó el brazo en una configuración rara, el global planner responde
   `failed to find a solution` y ni el paso de ir al inicio funciona.
2. **Paso 1** — agrega `pared_estatica`. Ya está cuando se planifica, así que el
   global planner la esquiva desde el primer plan.
3. **Paso 2** — manda la meta. Al 35% del camino recorrido inyecta
   `obstaculo_sorpresa` cruzando lo que queda de la trayectoria.

El disparo del paso 3 va por **progreso**, no por el primer feedback. El primer
feedback llega ~1 ms después de aceptar la meta, cuando el brazo todavía no se
movió: inyectar ahí hace que el local planner frene de una y el manager entre en
un bucle de replanificación sin haber ejecutado nada.

## Qué mirar en los logs

```
[local_planner_component]: Using 'trajectory_msgs/JointTrajectory' as local solution topic type
[hybrid_planning_manager]: Using planner logic interface 'moveit_hybrid_planning/ReplanInvalidatedTrajectory'
[hybrid_planning_manager]: Received goal request
[global_planner_component]: Received global planning goal request      <- plan global
[local_planner_component]: The local planner is solving...             <- arranca la ejecucion
>>> 35% del camino recorrido: aparece "obstaculo_sorpresa" ...
[local_planner_component]: Collision ahead, holding current position   <- lo detecta el local
[global_planner_component]: Received global planning goal request      <- el manager pide replan
```

Esa última línea repitiéndose **es** el hybrid planning: el global planner
volviendo a resolver mientras el local mantiene el robot quieto y seguro. Con
`SinglePlanExecution` en lugar de `ReplanInvalidatedTrajectory`, ahí abortaría.

## Estado: qué está verificado y qué no

### Funciona y es repetible

Medido en la simulación, dos corridas seguidas **sin reiniciar Gazebo**:

- Los tres componentes cargan con sus plugins.
- El `xarm6_joint_group_position_controller` se spawnea y se activa, y el
  `xarm6_traj_controller` queda inactivo.
- El reset del Paso 0 tarda ~0.3 s y converge siempre. Ya no hace falta reiniciar
  Gazebo entre corridas: un controlador de posición se puede comandar de vuelta a
  casa desde cualquier configuración.
- Global plan → ejecución local → cambio de escena → el local planner lo detecta
  (`Collision ahead, holding current position`) y **frena el brazo antes de
  chocar** → el manager termina.
- Determinista: 1 pedido de replanificación, ~10 s de punta a punta, sin bucles ni
  timeouts.

Eso ya es una demo válida de la arquitectura: se ve el planner global resolviendo,
el local ejecutando a 100 Hz, y el local reaccionando a un cambio del mundo que el
global no conocía.

### Lo que no funciona: "replanifica y sigue"

Ninguno de los dos planner logic plugins que trae MoveIt cierra el ciclo acá.

**`SinglePlanExecution`** (el default de este ejemplo) termina con:

```
'Single-Plan-Execution' plugin cannot handle events given as string.
```

No sabe qué hacer con `LOCAL_PLANNER_STUCK`. Termina rápido y sin ensuciar, que
es justo lo que se quiere para clase, pero el mensaje final es una limitación del
plugin y no un "aborté porque cambió el entorno".

**`ReplanInvalidatedTrajectory`** intenta replanificar, y se autolimita:

- **2548** pedidos de replanificación al global planner en 120 s, o sea ~21 por
  segundo.
- Cada pedido nuevo **preempta** el anterior. Con `allowed_planning_time = 0.5`,
  cada uno se corta a los ~47 ms: **ninguno llega a terminar**, así que nunca
  aparece una trayectoria nueva y el local planner se queda frenado para siempre
  (10390 `Collision ahead` en esa corrida, y timeout).

Para probarlo, cambiar una línea de `hybrid_planning_manager.yaml`:

```yaml
planner_logic_plugin_name: "moveit_hybrid_planning/ReplanInvalidatedTrajectory"
```

Es interesante mostrarlo en clase justamente por eso: se ven los pedidos de
replanificación en el log, y se ve por qué un lazo reactivo necesita control de
flujo.

### Descartado como causa, con evidencia

- **La geometría de los obstáculos.** Se probaron 8 candidatos con
  `/check_state_validity` y `/plan_kinematic_path` de move_group, sin mover el
  robot. Con la escena post-cambio (las dos placas, sin la estática) existe plan
  desde el estado donde el brazo queda frenado hasta la meta, y los dos estados
  son válidos. Incluso con las tres placas a la vez existe plan.
- **`local_planning_frequency`.** Se probó 10 Hz y 50 Hz además de 100 Hz.
- **`allowed_planning_time`.** 5.0 s y 0.5 s.

### Cómo cerrarlo de verdad

Escribir un planner logic plugin propio contra `PlannerLogicInterface`, que:

1. no pida un plan global nuevo si ya hay uno en vuelo, y
2. no trate los eventos que no conoce como fatales.

Son ~40 líneas de C++ más un `pluginlib` export. Es la única forma de que el
ciclo "obstáculo aparece → replanifica → llega a la meta" cierre con esta
arquitectura.

### Cómo ajustar el obstáculo sorpresa

Si se quiere probar otra geometría, el TCP recorre este camino (FK sobre el
camino articular `START_JOINTS` → `GOAL_JOINTS`):

```
  0%  (+0.539, +0.000, +0.495)
 35%  (+0.470, +0.250, +0.448)   <- punto de inyeccion
 50%  (+0.402, +0.339, +0.429)
 75%  (+0.255, +0.444, +0.397)
100%  (+0.084, +0.486, +0.369)   <- meta
```

Un candidato sirve si cumple las tres:

1. el brazo **no** está en colisión con él en la configuración de inyección,
2. invalida alguna configuración del camino después de ese punto,
3. sigue existiendo plan hasta la meta desde la última configuración válida.

Las tres se pueden verificar sin mover el robot, con `/check_state_validity` y
`/plan_kinematic_path`, que es mucho más rápido que correr la demo entera.

### Otras cosas conocidas

- `Cannot find planning configuration for group 'xarm6' using planner 'RRTConnect'`
  es esperable: este repo no define un bloque `planner_configs` con parámetros en
  ningún `ompl_planning.yaml`. OMPL cae a `geometric::RRTConnect` con defaults,
  que es lo que se quería. `move_group` tira el mismo aviso.
- `Failed to reload controllers: controller_manager_ does not exist` viene del
  moveit_cpp del global planner armando su trajectory execution, que en esta
  arquitectura no se usa (comanda el local planner). Es inofensivo.
- Si se lanza el launch dos veces sin matar el anterior, quedan dos
  `hybrid_planning_manager` sobre la misma acción y aparece
  `Ignoring unexpected goal response. There may be more than one action server`,
  con resultados sin sentido (`error_code=99999 "Unknown event"`). Matar el
  contenedor entre corridas:
  `pkill -9 -f component_container_mt`.
