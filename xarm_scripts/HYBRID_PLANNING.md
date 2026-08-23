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

## Qué se adaptó del demo de MoveIt

El demo oficial es para el Panda y en C++. Cambios para el xArm6:

- `group_name: xarm6`.
- Los comandos del local planner van a `/xarm6_traj_controller/joint_trajectory`
  como `trajectory_msgs/JointTrajectory`, porque en Gazebo el xArm6 corre un
  `joint_trajectory_controller`. El demo del Panda publica `Float64MultiArray` a
  un `joint_group_position_controller`, que acá no existe.
- `robot_description`, SRDF y cinemática salen de `MoveItConfigsBuilder` de
  `uf_ros_lib`, igual que el resto del repo, en vez de rutas fijas al Panda.
- La config de OMPL se arma juntando `moveit_configs/ompl_planning.yaml` con
  `xarm6/ompl_planning.yaml`. Si se pasa sólo la primera, OMPL avisa
  `Cannot find planning configuration for group 'xarm6'`.
- El nodo del demo es Python, para que se lea en clase.

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

Verificado contra la simulación en vivo, varias corridas:

- Los tres componentes cargan con los plugins correctos.
- El handoff global → local funciona (`Global goal accepted` → `Local goal
  accepted` → `The local planner is solving...`).
- El local planner publica al controlador del xArm6.
- La inyección por progreso dispara exactamente al 35%.
- `stop_before_collision` funciona: 606 `Collision ahead, holding current
  position` en una corrida.
- El manager pide replanificación al global planner: 14 pedidos tras la
  inyección.

**No funciona todavía:** el camino feliz completo, o sea obstáculo aparece →
replanifica → llega a la meta. Termina en
`Global planner failed to find a solution`. La causa está identificada y **no es
la geometría del obstáculo ni esta configuración**.

### Limitación conocida: el bucle de replanificación se pisa a sí mismo

En el log, antes del fallo, aparece:

```
[hybrid_planning_manager]: Hybrid Planning Manager failed to react to 'Global planning action aborted'
```

La secuencia es:

1. `forward_trajectory.cpp` manda `COLLISION_AHEAD` **una vez** por trayectoria
   (flag `path_invalidation_event_send_`).
2. `ReplanInvalidatedTrajectory::react()` responde a ese evento pidiéndole al
   global planner una meta nueva.
3. Cuando llega la solución global nueva, el local planner **resetea el flag**.
4. Si la trayectoria nueva sigue bloqueada, vuelve a mandar `COLLISION_AHEAD`, y
   se pide otra meta global.
5. Cada meta nueva **aborta** la anterior si todavía estaba resolviendo.
6. `ReplanInvalidatedTrajectory::react()` sólo maneja `COLLISION_AHEAD` y
   `LOCAL_PLANNER_STUCK`. Para cualquier otro evento devuelve
   `'ReplanInvalidatedTrajectory' plugin cannot handle this event` con
   `FAILURE`, y el hybrid planning entero muere.

O sea: el bucle de replanificación se autolimita. Bajar
`allowed_planning_time` a 0.5 s reduce la ventana pero no elimina la carrera.

Descartado como causa, con evidencia:

- **La geometría del obstáculo.** Se probaron 8 candidatos con los servicios
  `/check_state_validity` y `/plan_kinematic_path` de move_group, sin mover el
  robot. Cuatro cumplen las tres condiciones: el brazo no está en colisión al
  inyectar, el obstáculo invalida configuraciones del camino, y sigue existiendo
  plan hasta la meta — tanto desde el 35% como desde la última configuración
  válida antes del bloqueo. El que está en el código (`0.06 x 0.14 x 0.18` en
  `(0.26, 0.44, 0.40)`) es uno de esos cuatro.
- **`local_planning_frequency`.** Se probó a 10 Hz; misma carrera.

### Caminos para resolverlo

1. **Escribir un planner logic plugin propio.** Es la solución de fondo: una
   variante de `ReplanInvalidatedTrajectory` que ignore
   `Global planning action aborted` en vez de tratarlo como fatal, y que no pida
   una meta global nueva si ya hay una en vuelo. Son ~40 líneas de C++ contra
   `PlannerLogicInterface`.
2. **Usar `SinglePlanExecution`** y cambiar el objetivo de la demo: mostrar que
   sin replanificación el local planner igual frena el brazo antes de chocar, y
   el manager aborta con un error definido. Es una lección válida — la mitad
   "qué pasa sin hybrid planning" — y no depende de la parte frágil. Se cambia
   una línea de `hybrid_planning_manager.yaml`.

Para la clase, la opción 2 más el log de la opción 1 (14 pedidos de replan
visibles) cuenta la historia completa sin depender de que el camino feliz cierre.

### Reiniciar entre corridas

**Hay que reiniciar Gazebo entre corridas de la demo.** Durante el bucle de
frenar-y-replanificar el brazo termina clavado contra sus límites articulares
(se ven los valores exactos del URDF en `/joint_states`: `-2.059` de `joint2`,
`0.19198` de `joint3`, `-1.69297` de `joint5`) y trabado físicamente contra la
mesa. Desde ahí **ningún comando de posición lo saca** — se comprobó con 27 s de
comandos al controlador sin que se moviera un grado — y el global planner
responde `failed to find a solution` porque el estado inicial está fuera de
límites. El Paso 0 de la demo detecta esto y aborta con un mensaje claro.

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
