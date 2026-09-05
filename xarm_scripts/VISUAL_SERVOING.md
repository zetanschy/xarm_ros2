# Visual servoing con el xArm6 — Tarea 2 (Clase 5)

Notas de la infraestructura de la Tarea 2: qué se agregó, por qué está armado
así, y qué cosas ya se rompieron y están resueltas. Todo lo de acá está
verificado corriendo, no deducido.

## Qué se agregó

| Archivo | Qué es |
|---|---|
| `xarm_description/urdf/camera/wrist_camera.urdf.xacro` | Cámara de muñeca, coaxial con `link_tcp` |
| `xarm_gazebo/scripts/gen_aruco_world.py` | Genera el mundo: robot móvil omni + cubo con ArUco dibujado con geometría |
| `xarm_gazebo/worlds/table_aruco.world` | **Generado** — no editar a mano |
| `xarm_scripts/config/visual_servo/xarm6_servo.yaml` | Config de MoveIt Servo para Gazebo |
| `xarm_scripts/launch/visual_servo.launch.py` | Reset + pose de observación + cambio de controlador + servo + marcador |
| `xarm_scripts/xarm_scripts/aruco_target_mover.py` | Conduce el robot móvil: rectángulo y estacionamiento |

Se tocaron además `xarm_device_macro.xacro`, `xarm_device.urdf.xacro`,
`uf_ros_lib/moveit_configs_builder.py`, `_robot_moveit_gazebo.launch.py` y
`_robot_beside_table_gazebo.launch.py` para pasar el flag `add_wrist_camera` y
puentear los tópicos nuevos. La cámara se prende sola cuando el nombre del mundo
contiene `aruco`.

## Correr

```bash
# Terminal 1
ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py \
    world:=table_aruco.world add_gripper:=true

# Terminal 2
ros2 launch xarm_scripts visual_servo.launch.py mode:=medium
```

`mode`: `static` | `slow` | `medium` | `fast`. Con `move_target:=false` no se
lanza el nodo que mueve el marcador.

## Decisiones que no son obvias

### La pose de observación se subió con el robot

Con el Avular el marcador viaja mucho más alto que antes (cubierta en 1.108 contra
la mesa en 1.015). La pose de observación vieja dejaba la cámara a solo 9 cm del
marcador: cubría menos mesa que el propio recorrido, y el marcador se salía de
cuadro. Ahora el TCP va a 0.265 (link6 en 0.437), con la cámara ~0.18 m sobre el
marcador. Si se cambia la altura del robot o del objeto, esta pose hay que
recalcularla.

### La cámara es coaxial con el TCP, no va al costado

Primero estaba montada al costado del gripper, que es lo realista. El problema es
que entonces "marcador centrado en la imagen" significa "la **cámara** está sobre
el marcador", y el TCP queda desviado esa misma distancia — medido, unos 15 cm.
Peor: el desvío en píxeles depende de la altura, así que no se arregla con un
offset fijo en el setpoint. El alumno tendría que estimar profundidad solo para
poder centrar, que es exactamente la parte que esta tarea quiere evitar.

Montada a 0.12 m sobre `link_eef`, la cámara pasa el cuerpo del gripper y queda
entre los dedos, sobre el eje del `link_tcp` (que está a 0.172). Verificado por
TF: con `link6` en `(-0.300, -0.400, 0.350)`, la cámara queda en
`(-0.300, -0.400, 0.230)` — mismo x, mismo y.

### El campo de visión es de 86°, no de 60°

Con 60° el marcador de 36 mm desborda el cuadro a partir de unos 12 cm de
distancia, y el detector lo pierde justo en el tramo final del descenso. Con
1.5 rad entra entero hasta 3 cm y a 20 cm todavía mide unos 60 px.

### El marcador ArUco es geometría, no una textura

Cada celda negra del patrón es una caja de 6 mm sobre una placa blanca, generadas
por `gen_aruco_world.py` a partir del patrón real de `cv2.aruco`. Es más verboso
que un `albedo_map`, pero no depende de que `IGN_GAZEBO_RESOURCE_PATH` esté bien
(el launch exporta `GZ_SIM_RESOURCE_PATH`, que en Fortress no es la variable que
se lee) ni del mapeo UV de las caras de un `<box>`. Renderiza nítido y el
detector lo agarra sin problema.

### El objeto lo transporta un Avular Origin One

Las mallas son las reales, de
[avular_origin_simulation](https://github.com/zetanschy/avular_origin_simulation)
(`origin_one_description`), copiadas a `xarm_gazebo/models/avular_origin/meshes/`
para que el mundo no dependa de tener ese repo clonado. Están en milímetros, de
ahí el `0.001` del factor de escala.

**Escaladas a 0.30.** A tamaño real el Origin mide 656 × 408 × 264 mm con ruedas
mecanum de 202 mm: sobre esta mesa chocaría con la base del brazo y ocuparía casi
todo el ancho útil. A 0.30 queda en 197 × 122 × 79 mm, entra cómodo, y se sigue
leyendo como el mismo robot. Las medidas del chasis (track 475 mm, wheelbase
410 mm, ground clearance 45.5 mm, eje a 56 mm) salen del
`origin_one.urdf.xacro` original, escaladas igual.

Recorre un rectángulo de **220 × 160 mm**, da tres vueltas, y se estaciona en el
centro. Recién ahí el brazo baja a sacarle el cubo.

**Se mueve con dos juntas prismáticas (X, Y) controladas por posición, no con las
ruedas.** Las ruedas mecanum son solo visuales y no giran. Que sean mecanum no es
decorativo: el recorrido incluye tramos laterales, y una base con ruedas normales
no podría hacerlos. Simular la tracción sería peor para esta tarea: el recorrido
dependería del agarre, y una patinada deja al robot fuera del alcance del brazo o
fuera del campo de visión.

La colisión es una caja, no la malla. Una malla de 125 mil vértices como geometría
de colisión es un costo enorme para lo único que hace falta: que el cubo se apoye
en una cubierta plana.

Y va por **posición, no por velocidad**. Con comandos de velocidad la posición
queda a lazo abierto: se integra v·dt, el error se acumula, y a los pocos ciclos
el robot se clava contra el tope de una junta con el marcador quieto y los
comandos alternando sin efecto. Como los tópicos se ven perfectos, cuesta darse
cuenta.

**El rectángulo entra entero en el campo de visión.** Es la medida que hace que la
tarea sea posible: desde la pose de observación la cámara cubre ±0.164 m en
horizontal y ±0.121 en vertical a la altura del marcador, contra ±0.08 y ±0.06 de
recorrido. Con el riel anterior (±0.12 m) el marcador se salía de cuadro al
descender y el brazo se quedaba sin referencia. Verificado: 12+ s de seguimiento
continuo sin una sola pérdida.

### La superficie de la mesa está en 1.015, no en 1.00

Esto costó caro. La bandeja original tenía el piso en 1.010, o sea **por debajo**
de la superficie de apoyo real: el cubo se apoyaba en la mesa, la bandeja le
pasaba por abajo sin tocarlo, y **el marcador no se movía nunca** aunque el carro
sí. Las primeras mediciones de seguimiento y los primeros "4 de 4 agarres" se
hicieron, sin saberlo, sobre un objeto quieto.

La lección: la altura de apoyo se mide, no se calcula. Se deja caer el cubo sobre
la superficie y se lee la pose viva. Los espesores del `.world` no alcanzan porque
el contacto deja que el objeto se hunda unos milímetros.

### El launch espera a la simulacion, y aborta si un paso falla

Lanzar la Terminal 2 antes de que Gazebo termine de levantar los controladores es
lo que va a pasar siempre: Gazebo tarda entre 60 y 90 s. La primera version del
launch se tragaba todos los errores con `|| true` y seguia igual -- publicaba la
trayectoria a la pose de observacion sin que nadie la ejecutara, cambiaba de
controlador, levantaba el servo, y dejaba el brazo en la pose de spawn **sin un
solo mensaje**. El sintoma es "no detecta el marcador", que manda a buscar el
problema al codigo de vision del alumno.

Dos cambios:

- El paso 0 espera (hasta 180 s) a que `xarm6_traj_controller` aparezca en
  `ros2 control list_controllers`, y si no aparece corta con un mensaje que dice
  que hay que arrancar la Terminal 1 primero.
- El movimiento a la pose de observacion va por la **accion**
  `/xarm6_traj_controller/follow_joint_trajectory`, no por el topico
  `joint_trajectory`. La accion bloquea hasta que el brazo llega y dice si el
  goal fue aceptado; el topico no responde nada. Si el resultado no es
  `SUCCEEDED`, el paso corta.

Ademas, si cualquiera de los pasos encadenados sale con codigo distinto de cero,
el launch hace `Shutdown()` en vez de seguir. Levantar el servo sobre un brazo
mal posicionado solo sirve para que el error aparezca mas tarde y disfrazado.

Verificado: con Gazebo recien arrancado y la Terminal 2 lanzada 3 s despues, el
launch espera, resetea, y el TCP queda en `(-0.300, -0.400, 0.178)`.

### Relanzar tiene que funcionar sin reiniciar Gazebo

En clase se relanza mucho. Dos cosas lo hacían fallar y están resueltas:

- El paso 1 reactiva el `xarm6_traj_controller` antes de mandar la trayectoria a
  la pose de observación. Si no, quedó inactivo del paso 2 de la corrida
  anterior, la trayectoria se publica, nadie la ejecuta, y el brazo se queda
  donde estaba.
- El paso 0 resetea la escena: abre el gripper, manda el carro al centro, y
  teletransporta el cubo a la bandeja. Sin esto, después de un agarre exitoso el
  cubo queda colgando del gripper y al mover el brazo cae fuera de la bandeja
  para no volver nunca.

El orden del reset importa: gripper primero, después el carro al centro, y recién
ahí el cubo. Teletransportar el cubo con el carro en un extremo lo deja fuera de
la bandeja.

## El agarre: por qué el cubo se resbalaba

Esto costó varias iteraciones. Los síntomas eran engañosos: en la cámara de
muñeca el agarre se veía **perfecto**, con el marcador centrado entre los dos
dedos, y el brazo subía con las manos vacías.

**Causa 1 — fricción.** El cubo tenía `mu = 2.0`, un valor físicamente sensato.
No alcanza. El `small_box` de `table_gz.world`, que es el que agarra
`pick_and_place.py`, usa `mu = 1000`, fricción torsional, `kp = 1e8` y
`min_depth = 0.002`. No es realismo: es el truco necesario para que un gripper
paralelo controlado por posición sostenga un objeto libre en Gazebo. El cubo del
ArUco ahora copia ese bloque al pie de la letra.

**Causa 2 — cuánto se cierra el gripper.** `drive_joint` va de 0 (abierto) a 0.85
(cerrado del todo). La intuición dice "cerrá más para agarrar mejor", y es al
revés: con 0.60 y con 0.85 el cubo se queda en la bandeja; con **0.50** sube. Los
dedos son juntas controladas por posición y siguen empujando más allá del
contacto; el solver resuelve la penetración expulsando el cubo. 0.50 deja la
apertura útil en unos 40 mm, justo el ancho del cubo. Es el mismo valor que usa
`pick_and_place.py`, y ahora se entiende por qué.

Medido en esta simulación, separación **entre los frames** de los dedos:

| `drive_joint` | separación |
|---|---|
| 0.00 | 141 mm |
| 0.40 | 103 mm |
| 0.50 | 92 mm |
| 0.60 | 81 mm |
| 0.85 | 54 mm |

Los frames están unos 25 mm por fuera de la superficie de contacto de cada lado,
así que hay que restar unos 50 mm para tener la apertura útil.

**Causa 3 — la velocidad de ascenso.** Al acelerar el descenso subí `MAX_Z_SPEED`
a 0.25 m/s, y el ascenso usaba la misma constante. El tirón rompe el agarre. Son
dos constantes distintas por una razón física, no por descuido. Medido, subiendo
desde la cubierta del Avular:

| Velocidad de ascenso | Resultado |
|---|---|
| 0.25 m/s | sube 15 mm y se cae |
| 0.06 m/s | sube 68 mm, se sostiene medio segundo, resbala |
| 0.04 m/s | sube 88 mm y **aguanta** |

Y hay que darle tiempo al contacto: 2 s entre cerrar el gripper y empezar a subir.
La trayectoria del gripper dura 1 s, así que con menos margen se arranca a subir
con los dedos todavía moviéndose, y eso es medio agarre.

## Trampa al verificar: `ign model --pose` miente

`ign model --model aruco_cube --pose` devuelve una pose **cacheada**. Durante el
desarrollo dio `(0.200, -0.800, 1.040)` mientras la cámara mostraba la bandeja
vacía y el cubo tirado al costado de la mesa. Varias conclusiones intermedias
salieron mal por esto.

La pose viva sale de:

```bash
ign topic -e -t /world/default/dynamic_pose/info
```

Lo mismo para el carro: `ign model --model aruco_cart --pose` devuelve la pose
del **modelo**, que es fija. El que se mueve es el link:
`ign model --model aruco_cart --link cart`.

## El lazo P no alcanza a un blanco que se mueve

Es el punto de control de la tarea. Un proporcional puro deja un error de régimen
permanente frente a una rampa, igual a `v_blanco / K` con `K = KP / altura`.
Medido con el marcador a 0.06 m/s (`mode:=medium`) y la cámara a ~0.19 m:

| `KP_XY` | error mediano | máximo | tiempo bajo 40 px |
|---|---|---|---|
| 0.1 | 117 px | 264 px | 14 % |
| 0.6 | 39 px | 66 px | 51 % |
| 1.2 | 20 px | 34 px | 100 % |

El umbral del Ejercicio 2 (40 px sostenidos 10 s) está fijado a partir de esta
tabla: es holgado con `KP_XY = 1.2` e imposible con la ganancia baja con la que
todos empiezan.

## Estado

Verificado corriendo, en este orden:

- La cámara publica a 30 Hz en `/wrist_camera/image_raw`, con intrínsecos en
  `/wrist_camera/camera_info` (`fx = 343.5`).
- El servo mueve el brazo con twists en `wrist_camera_optical_frame`; `+z` de ese
  frame baja hacia la mesa.
- Las mallas del Avular cargan sin un solo error y el cubo reposa exactamente en
  1.1326, sobre la cubierta.
- Las cuatro esquinas del rectángulo de 220 × 160 mm son alcanzables (las cuatro
  planifican y el brazo llega con ±2 mm), y desde todas ellas el marcador se
  sigue detectando a 68 px.
- Seguimiento durante el rectángulo completo de 220 × 160 mm: recorrido medido
  0.222 m y **cero pérdidas de marcador** en toda la corrida.
- Agarre y levantamiento del cubo estacionado: 88 mm, y se queda arriba.
- Relanzar la Terminal 2 sin reiniciar Gazebo deja el brazo otra vez en
  `(-0.300, -0.400, 0.178)` y el cubo sobre el robot.

## Pendiente

- **`mode:=fast` (0.12 m/s) no está probado a fondo.** El enunciado sólo exige
  `slow` y `medium`. Si se quiere usar `fast` como desafío opcional, hay que
  medir antes si el lazo lo aguanta.
- El `Sensors` de Ignition **no** se declara en `wrist_camera.urdf.xacro` a
  propósito: va en el `.world`. Declararlo en los dos lados lo carga dos veces,
  Fortress construye la escena por duplicado y el hilo de render se cae con
  SIGSEGV al arrancar (`Visual [x] already exists`). Si alguien usa la cámara de
  muñeca en otro mundo, ese mundo tiene que traer el plugin `Sensors`.
