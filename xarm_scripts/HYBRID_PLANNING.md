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

Verificado contra la simulación en vivo:

- Los tres componentes cargan con los plugins correctos.
- El handoff global → local funciona (`Global goal accepted` → `Local goal
  accepted` → `The local planner is solving...`).
- El local planner publica al controlador del xArm6.
- La inyección por progreso dispara exactamente al 35%.
- `stop_before_collision` funciona: 694 `Collision ahead, holding current
  position` en una corrida.
- El manager pide replanificación al global planner: 7 pedidos tras la inyección.

**No verificado todavía:** el camino feliz completo, o sea obstáculo aparece →
replanifica → llega a la meta. En todas mis corridas terminó en
`Global planner failed to find a solution`, porque el obstáculo sorpresa deja al
brazo sin salida: cae sobre el volumen que sus propios eslabones ya ocupan, y
entonces el estado inicial del replan queda en colisión.

Eso es **geometría, no arquitectura** — toda la maquinaria reactiva ya se ve
funcionando en los logs. Para ajustarlo hace falta ver RViz mientras corre, que
es justo lo que no pude hacer acá (rviz2 no arranca desde una shell del snap de
VS Code).

### Cómo ajustar el obstáculo sorpresa

El TCP recorre este camino (FK sobre el camino articular `START_JOINTS` →
`GOAL_JOINTS`):

```
  0%  (+0.539, +0.000, +0.495)
 35%  (+0.470, +0.250, +0.448)   <- punto de inyeccion
 50%  (+0.402, +0.339, +0.429)
 75%  (+0.255, +0.444, +0.397)
100%  (+0.084, +0.486, +0.369)   <- meta
```

Reglas que salieron de las pruebas:

- **Chico, no una pared.** Un obstáculo grande no deja vuelta y el global planner
  devuelve `failed to find a solution` en vez de replanificar.
- **Bien por delante del punto de inyección.** Si cae donde el brazo ya está, el
  estado inicial queda en colisión.
- **Lejos de la meta.** Si tapa la zona de la meta, no hay solución posible.

Empezar chico (0.06 × 0.14 × 0.18) alrededor del 60–75% del camino y agrandar
hasta que invalide la trayectoria pero siga habiendo camino.

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
