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
| Planner logic | `xarm_hybrid_planning/ReplanWhenIdle` (de este repo) | `config/hybrid_planning/hybrid_planning_manager.yaml` |
| Global planner | `moveit_hybrid_planning/MoveItPlanningPipeline` | `config/hybrid_planning/global_planner.yaml` |
| Trajectory operator | `moveit_hybrid_planning/SimpleSampler` | `config/hybrid_planning/local_planner.yaml` |
| Local constraint solver | `moveit_hybrid_planning/ForwardTrajectory` | `config/hybrid_planning/local_planner.yaml` |

El logic plugin es el interesante para la clase. Este ejemplo usa uno propio,
`xarm_hybrid_planning/ReplanWhenIdle`, porque **ninguno de los dos que trae MoveIt
cierra el ciclo reactivo acá**. Ver "Arreglo B" más abajo para el detalle, y la
tabla de contraste para qué hace cada uno.

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

## Estado: funciona de punta a punta

El ciclo completo cierra: **obstáculo aparece → replanifica → llega a la meta**.
Dos corridas seguidas sin reiniciar Gazebo, idénticas:

```
Pidiendo plan global (intento 1 de 30).
>>> Salio la primera solucion global. Ahora cambia la escena...
Pidiendo plan global (intento 2 de 30).
Pidiendo plan global (intento 3 de 30).
Hybrid planning termino OK.
```

El brazo llega a la meta con 0.022 rad de error total sobre las 6 articulaciones.
Tres replanificaciones, ~10 s de punta a punta.

Para que llegara ahí hicieron falta **dos arreglos más** además de la interfaz de
control. Los dos son didácticos y vale la pena contarlos en clase.

### Arreglo A: el global planner no veía la escena

En la config de `moveit_cpp` los dos nombres de tópico están **al revés** de lo
que uno espera, y el comentario del config de referencia de MoveIt lo dice
explícito:

```yaml
planning_scene_monitor_options:
  publish_planning_scene_topic: "/monitored_planning_scene"    # al que SE SUSCRIBE
  monitored_planning_scene_topic: "/global_planner/planning_scene"  # el que PUBLICA
```

Con `publish_planning_scene_topic: "/planning_scene"`, el global planner **no veía
los objetos** que la demo agrega por el servicio `/apply_planning_scene`.
Planificaba derecho a través de las placas, terminaba en ~40 ms, y el local
planner (que sí las ve, por `startSceneMonitor`) rechazaba la trayectoria una y
otra vez. Bucle infinito sin que nada reportara error.

Síntoma para reconocerlo: el global planner "resuelve" sospechosamente rápido y
siempre, mientras el local planner nunca acepta la trayectoria.

### Arreglo B: el lazo reactivo necesita control de tasa, no sólo exclusión mutua

El plugin propio (`xarm_hybrid_planning/ReplanWhenIdle`, paquete
`xarm_hybrid_planning` de este repo) hace tres cosas que
`ReplanInvalidatedTrajectory` no hace:

1. **Exclusión mutua**: no pide un plan global si ya hay uno en vuelo. Sin esto,
   cada evento del local planner dispara un pedido que preempta al anterior y
   ninguno termina (medido: 2548 pedidos en 120 s con el plugin original).
2. **Límite de tasa**: 500 ms mínimo entre pedidos. La exclusión mutua sola no
   alcanza, porque el global planner resuelve en ~40 ms y `LOCAL_PLANNER_STUCK` se
   re-arma cada pocas decenas de ms mientras el brazo esté quieto
   (`forward_trajectory.cpp` resetea `num_iterations_stuck_` al emitirlo). Sin
   límite de tasa se gastan 10 reintentos en 1.3 s **aunque el brazo esté
   avanzando bien**.
3. **Un aborto de la acción global no es fatal** una vez que la ejecución
   arrancó: casi siempre es un pedido nuestro preemptando al anterior. Y ningún
   evento desconocido aborta el hybrid planning, a diferencia de los dos plugins
   de MoveIt.

Más un tope de 30 reintentos para que la demo termine si de verdad no hay salida.

### Los dos plugins de MoveIt, para contrastar en clase

Cambiando una línea de `hybrid_planning_manager.yaml`:

| Plugin | Qué pasa |
|---|---|
| `xarm_hybrid_planning/ReplanWhenIdle` | cierra el ciclo, 3 replanificaciones, `termino OK` |
| `moveit_hybrid_planning/SinglePlanExecution` | no replanifica; termina con `cannot handle events given as string` al recibir `LOCAL_PLANNER_STUCK` |
| `moveit_hybrid_planning/ReplanInvalidatedTrajectory` | se autolimita: ~21 pedidos/s, ninguno termina, y el primer aborto mata todo |

Es una buena demo de por qué un lazo reactivo necesita control de flujo.

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

### Por qué el movimiento se ve raro

Se frena en seco, arranca, cambia de dirección y a veces va a tirones. Cuatro
causas, todas identificadas en los logs y en el código de MoveIt. Las dos
primeras son **el comportamiento correcto**, no fallas:

**1. Se frena en seco durante ~1.7 s.** Cuando aparecen las placas, el local
planner deja de avanzar y mantiene la posición (`Collision ahead, holding current
position`, decenas de veces) mientras el manager pide planes nuevos. Eso es
literalmente `stop_before_collision: true` haciendo su trabajo: el brazo no sigue
hacia un obstáculo, se queda quieto hasta tener una ruta válida.

**2. Cambia de dirección entre replanificaciones.** Cada plan global es una
solución nueva de RRTConnect, que es aleatorizado. Si el brazo alcanza a moverse
un poco con el plan N y después llega el plan N+1 por otra ruta, se ve como una
corrección brusca. Con 3-4 replanificaciones seguidas, se ven 3-4 correcciones.

**3. La temporización de la trayectoria no se respeta.** Esta es la que más
sorprende. `ForwardTrajectory` le pasa al controlador **el siguiente waypoint** que
le da `SimpleSampler`, y el `JointGroupPositionController` escribe esa posición tal
cual: **no interpola**. Así que el perfil de velocidad que calculó el planner no se
usa; la velocidad real sale de qué tan espaciados quedaron los waypoints y de qué
tan rápido el sampler avanza el índice. El demo del Panda de MoveIt tiene la misma
propiedad.

**4. Dos avisos que dejan los planes más gruesos.** `Cannot find planning
configuration for group 'xarm6'` (este repo no define `planner_configs` en ningún
`ompl_planning.yaml`, así que no hay `longest_valid_segment_fraction` propio) y
`Joint acceleration limits are not defined. Using the default 1 rad/s^2` (los
límites sí están en `joint_limits.yaml` pero no llegan al time parameterization del
global planner). Los dos hacen que las trayectorias salgan menos finas.

#### Lo que NO conviene hacer: bajar el escalado de velocidad

Parece la solución obvia y **rompe la demo**. Con
`max_velocity_scaling_factor = 0.15` el movimiento sale mucho más parejo, pero el
siguiente waypoint queda siempre tan cerca del estado actual que `isPathValid`
nunca lo encuentra en colisión: `stop_before_collision` no dispara nunca. Medido:
**0** `Collision ahead` en dos corridas, y el brazo llega a la meta sin reaccionar
a nada. La demo pasa a no demostrar nada.

Por eso el valor queda en `0.4`. Si se quiere movimiento más suave sin perder la
reacción, hay que atacar la causa 3 (que el controlador respete la temporización),
no bajar la velocidad.

### Relanzar el ejemplo

Ctrl-C en la terminal 2 y volver a lanzar. El launch mata a sus propios hijos,
pero si quedó algo colgado:

```bash
pkill -9 -f component_container_mt
pkill -9 -f hybrid_planning_demo
```

Matar **los dos**. Si queda vivo el demo de la corrida anterior, la siguiente
tiene dos nodos con el mismo nombre sobre la misma acción y el resultado no tiene
sentido (se reconoce por el warning `Publisher already registered for provided
node name`).

El paso que cambia los controladores es idempotente: si el
`xarm6_joint_group_position_controller` ya está cargado lo reactiva en vez de
fallar. En el log se ve `position controller ya cargado; se reactiva`.

### Si el contenedor se cae con SIGSEGV

Se vio una vez un `component_container_mt ... exit code -11` alrededor de un
segundo después de aceptar la meta. No se pudo reproducir en 6 corridas
posteriores, así que por ahora queda como intermitente y sin diagnóstico.

Para que la próxima vez deje algo utilizable, instalar el handler de señales que
imprime el stack trace:

```bash
sudo apt install ros-humble-backward-ros
```

Con eso, un crash en cualquier nodo de MoveIt escribe el backtrace en
`~/.ros/log/<timestamp>/`. Sin él, el directorio del launch sólo tiene
`launch.log` con la línea `process has died` y nada más — que es exactamente lo
que pasó.

El container hospeda los tres componentes, así que un SIGSEGV se lleva los tres.
Si molesta para depurar, se pueden lanzar como nodos separados en vez de
componibles (a costa de latencia de IPC en el lazo del local planner, que es la
razón por la que MoveIt los compone).

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
