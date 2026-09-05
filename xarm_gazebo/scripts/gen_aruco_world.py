#!/usr/bin/env python3
"""Genera xarm_gazebo/worlds/table_aruco.world (Tarea 2 - visual servoing).

La escena: un robot movil chico recorre un rectangulo sobre la mesa llevando
encima un cubo con un marcador ArUco. Cuando termina su recorrido se estaciona, y
recien ahi el brazo baja y le saca el cubo de encima.

OJO con los comentarios XML del mundo: no pueden contener dos guiones seguidos.
Gazebo lo tolera, pero cualquier parser estricto (xml.dom.minidom, editores,
linters) rechaza el archivo entero. Usar ":" o "," en su lugar.

El marcador ArUco se dibuja con geometria, no con una textura: cada celda negra
del patron es una caja delgada sobre una placa blanca. Es mas verboso que un
albedo_map, pero no depende de IGN_GAZEBO_RESOURCE_PATH ni del mapeo UV de las
caras de un <box>, que es donde se rompen las texturas en Fortress.

Uso:  python3 gen_aruco_world.py            # regenera el .world in situ
"""

import os

import cv2

MARKER_ID = 0
DICT = cv2.aruco.DICT_4X4_50
CELLS = 6          # 4x4 bits + 1 celda de borde por lado
CELL = 0.006       # m por celda -> marcador de 36 mm
MARKER_Z = 0.0004  # espesor de las celdas negras sobre la placa

# Objeto agarrable. 40 mm de ancho porque es el valor con el que el gripper
# cierra bien (drive_joint = 0.50), y 50 de alto porque con el TCP a la altura de
# su centro las almohadillas de los dedos lo tocan desde ahi hasta arriba: con 30
# mm de alto quedan solo 15 mm de contacto y el agarre se resbala.
CUBE_XY = 0.04
CUBE_Z = 0.05

# La tapa de la mesa de Fuel NO esta en 1.00. Medido dejando caer el cubo sobre la
# mesa desnuda y leyendo la pose viva, la superficie de apoyo esta en 1.015.
TABLE_SURFACE = 1.015
MODEL_Z = 1.00     # origen del modelo del robot movil

# Robot movil: mallas reales del Avular Origin One, de
# https://github.com/zetanschy/avular_origin_simulation (origin_one_description).
# Las mallas estan en MILIMETROS, de ahi el 0.001 del factor de escala.
#
# A tamano real el Origin mide 656 x 408 x 264 mm y sus ruedas mecanum 202 mm de
# diametro: sobre esta mesa chocaria con la base del brazo y ocuparia casi todo
# el ancho util. SCALE lo deja en 197 x 122 x 79 mm, que entra comodo y sigue
# leyendose como el mismo robot.
SCALE = 0.30
MESH_SCALE = 0.001 * SCALE

# Medidas del Origin real, del origin_one.urdf.xacro, escaladas.
TRACK_WIDTH = 0.475 * SCALE
WHEELBASE = 0.410 * SCALE
GROUND_CLEARANCE = 0.0455 * SCALE
WHEEL_Z = 0.056 * SCALE          # altura del eje respecto del cuerpo
BODY_LEN = 0.656 * SCALE
BODY_WID = 0.408 * SCALE
BODY_HEIGHT = 0.26327 * SCALE    # extension en z de la malla del cuerpo

# El cuerpo apoya a GROUND_CLEARANCE sobre la mesa; su cara de arriba es la
# cubierta donde viaja el cubo.
BODY_BOTTOM = TABLE_SURFACE + GROUND_CLEARANCE
BODY_TOP = BODY_BOTTOM + BODY_HEIGHT

# Recorrido rectangular, en coordenadas del MUNDO y respecto del centro (0.2, -0.8).
#
# Las medidas no son a ojo. El robot se spawnea con link_base en (-0.2, -0.5) y
# yaw +90 deg, asi que un punto (bx, by) de link_base cae en el mundo en
# (-0.2 - by, -0.5 + bx). Con estos semiejes las cuatro esquinas quedan a un radio
# de entre 0.40 y 0.60 m del hombro, que es el rango que ya se verifico alcanzable
# con el gripper apuntando hacia abajo.
#
# Y ademas el rectangulo entero entra en el campo de vision desde la pose de
# observacion: la camara cubre +-0.164 m en horizontal y +-0.121 en vertical a la
# altura del marcador, contra los +-0.11 y +-0.08 del recorrido. El brazo nunca
# pierde el marcador de vista aunque no lo siga bien.
RECT_HALF_X = 0.11
RECT_HALF_Y = 0.08
JOINT_MARGIN = 0.03   # holgura entre el recorrido y el tope mecanico


def marker_grid():
    """Devuelve la matriz CELLSxCELLS del marcador: True = celda negra."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICT)
    img = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, CELLS, borderBits=1)
    return [[img[r][c] == 0 for c in range(CELLS)] for r in range(CELLS)]


def build_world():
    marker_side = CELLS * CELL
    cube_top = CUBE_Z / 2.0
    cube_z = BODY_TOP + CUBE_Z / 2.0
    chassis_z = (BODY_BOTTOM + BODY_TOP) / 2.0 - MODEL_Z

    return f'''<?xml version="1.0" ?>
<!-- GENERADO por xarm_gazebo/scripts/gen_aruco_world.py, no editar a mano. -->
<sdf version="1.6">
  <world name="default">

    <!-- Sensors es obligatorio para que la camara de muneca renderice. Sin este
         plugin el topico /wrist_camera existe pero no publica nada. -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table</uri>
      <name>table</name>
      <pose>0.0 -0.84 0 0 0 0</pose>
    </include>

    <!-- Robot movil que transporta el cubo.

         Es un robot OMNI: se desplaza en X e Y sin girar, que es justo lo que
         permiten las dos juntas prismaticas. Las ruedas son decorativas y van a
         45 grados para que se lea como una base omni.

         No se simula la traccion a proposito. Con ruedas motrices el recorrido
         depende del agarre, y una patinada deja al robot fuera del alcance del
         brazo o fuera del campo de vision; con juntas por posicion el rectangulo
         es exacto y repetible, que es lo que necesita una tarea que se corrige.

         El chasis NO toca la mesa (queda 5 mm de aire): si rozara, la friccion
         pelearia contra las juntas. -->
    <model name="mobile_robot">
      <pose>0.2 -0.8 {MODEL_Z} 0 0 0</pose>
      <static>false</static>

      <link name="anchor">
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>1e-4</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-4</iyy><iyz>0</iyz><izz>1e-4</izz></inertia>
        </inertial>
      </link>
      <joint name="anchor_fixed" type="fixed">
        <parent>world</parent>
        <child>anchor</child>
      </joint>

      <link name="x_carriage">
        <pose>0 0 {chassis_z:.4f} 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>1e-4</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-4</iyy><iyz>0</iyz><izz>1e-4</izz></inertia>
        </inertial>
      </link>
      <joint name="slide_x" type="prismatic">
        <parent>anchor</parent>
        <child>x_carriage</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit>
            <lower>{-(RECT_HALF_X + JOINT_MARGIN):.3f}</lower>
            <upper>{RECT_HALF_X + JOINT_MARGIN:.3f}</upper>
            <effort>500</effort><velocity>1.0</velocity>
          </limit>
          <dynamics><damping>2.0</damping></dynamics>
        </axis>
      </joint>

      <link name="chassis">
        <pose>0 0 {chassis_z:.4f} 0 0 0</pose>
        <inertial>
          <mass>2.0</mass>
          <inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia>
        </inertial>
        <!-- Malla real del Avular Origin One. El origen de la malla coincide
             con el frame main_body del URDF original, que esta a
             GROUND_CLEARANCE del piso. -->
        <visual name="body">
          <pose>0 0 {-BODY_HEIGHT / 2:.4f} 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>model://avular_origin/meshes/body.obj</uri>
              <scale>{MESH_SCALE} {MESH_SCALE} {MESH_SCALE}</scale>
            </mesh>
          </geometry>
        </visual>
{_wheels()}
        <!-- La colision es una caja, no la malla: el cubo solo necesita apoyarse
             en una cubierta plana, y una malla de 125 mil vertices como geometria
             de colision es un costo enorme para nada. La caja llega justo hasta
             la cara de arriba del cuerpo. -->
        <collision name="deck">
          <geometry><box><size>{BODY_LEN:.4f} {BODY_WID:.4f} {BODY_HEIGHT:.4f}</size></box></geometry>
          <surface>
            <friction>
              <ode><mu>1000.0</mu><mu2>1000.0</mu2><slip1>0.0</slip1><slip2>0.0</slip2></ode>
              <torsional>
                <coefficient>100.0</coefficient>
                <use_patch_radius>true</use_patch_radius>
                <patch_radius>0.1</patch_radius>
                <surface_radius>0.05</surface_radius>
              </torsional>
            </friction>
            <contact>
              <ode><kp>1e8</kp><kd>1000</kd><max_vel>0.0</max_vel><min_depth>0.002</min_depth></ode>
            </contact>
          </surface>
        </collision>
      </link>
      <joint name="slide_y" type="prismatic">
        <parent>x_carriage</parent>
        <child>chassis</child>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>{-(RECT_HALF_Y + JOINT_MARGIN):.3f}</lower>
            <upper>{RECT_HALF_Y + JOINT_MARGIN:.3f}</upper>
            <effort>500</effort><velocity>1.0</velocity>
          </limit>
          <dynamics><damping>2.0</damping></dynamics>
        </axis>
      </joint>

      <plugin filename="ignition-gazebo-joint-position-controller-system"
              name="ignition::gazebo::systems::JointPositionController">
        <joint_name>slide_x</joint_name>
        <topic>/mobile_robot/cmd_x</topic>
        <p_gain>400.0</p_gain><i_gain>0.0</i_gain><d_gain>40.0</d_gain>
        <cmd_max>200.0</cmd_max><cmd_min>-200.0</cmd_min>
      </plugin>
      <plugin filename="ignition-gazebo-joint-position-controller-system"
              name="ignition::gazebo::systems::JointPositionController">
        <joint_name>slide_y</joint_name>
        <topic>/mobile_robot/cmd_y</topic>
        <p_gain>400.0</p_gain><i_gain>0.0</i_gain><d_gain>40.0</d_gain>
        <cmd_max>200.0</cmd_max><cmd_min>-200.0</cmd_min>
      </plugin>
    </model>

    <!-- Cubo con el marcador ArUco id {MARKER_ID} ({marker_side * 1000:.0f} mm) en la cara
         de arriba. Viaja apoyado sobre el robot, sujeto solo por friccion: por eso
         los parametros de superficie son los mismos, exagerados, que usa el
         small_box de table_gz.world, que es el cubo que agarra pick_and_place.py.
         Con valores fisicamente sensatos (mu = 2) el cubo se queda atras, y al
         agarrarlo se resbala de los dedos. -->
    <model name="aruco_cube">
      <pose>0.2 -0.8 {cube_z:.4f} 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <inertial>
          <mass>0.01</mass>
          <inertia><ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz></inertia>
        </inertial>
        <visual name="body">
          <geometry><box><size>{CUBE_XY} {CUBE_XY} {CUBE_Z}</size></box></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
        <visual name="marker_plate">
          <pose>0 0 {cube_top + 0.0002:.4f} 0 0 0</pose>
          <geometry><box><size>{marker_side + 2 * CELL:.4f} {marker_side + 2 * CELL:.4f} 0.0004</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
{_marker_cells(cube_top)}
        <collision name="collision">
          <geometry><box><size>{CUBE_XY} {CUBE_XY} {CUBE_Z}</size></box></geometry>
          <surface>
            <friction>
              <ode><mu>1000.0</mu><mu2>1000.0</mu2><slip1>0.0</slip1><slip2>0.0</slip2><fdir1>0 0 1</fdir1></ode>
              <torsional>
                <coefficient>100.0</coefficient>
                <use_patch_radius>true</use_patch_radius>
                <patch_radius>0.1</patch_radius>
                <surface_radius>0.05</surface_radius>
              </torsional>
            </friction>
            <contact>
              <ode><kp>1e8</kp><kd>1000</kd><max_vel>0.0</max_vel><min_depth>0.002</min_depth></ode>
            </contact>
            <bounce><restitution_coefficient>0</restitution_coefficient><threshold>0</threshold></bounce>
          </surface>
        </collision>
      </link>
    </model>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

  </world>
</sdf>
'''


def _wheels():
    """Las cuatro ruedas mecanum del Origin, con sus mallas reales.

    Son SOLO visuales, sin colision y sin girar: el robot se mueve con dos juntas
    prismaticas, que es movimiento holonomico puro. Justamente por eso las ruedas
    tienen que ser mecanum: un robot con ruedas normales no podria desplazarse de
    costado, y el rectangulo incluye tramos laterales.

    Simular la traccion de verdad seria peor para esta tarea: con ruedas motrices
    el recorrido depende del agarre, y una patinada deja al robot fuera del
    alcance del brazo o fuera del campo de vision.

    Las rotaciones salen de componer las dos que trae el URDF original: la del
    joint (rpy -1.5707 0 0) con la del visual (rpy 0 +-1.5707 0).
    """
    z = -BODY_HEIGHT / 2.0 + WHEEL_Z
    out = []
    for sx, fr in ((1, 'f'), (-1, 'r')):
        for sy, lr in ((1, 'l'), (-1, 'r')):
            x = sx * WHEELBASE / 2.0
            y = sy * TRACK_WIDTH / 2.0
            # Rueda A para el lado izquierdo, B para el derecho: los rodillos van
            # inclinados al reves en cada lado, que es lo que hace mecanum a una
            # base mecanum.
            mesh = 'wheel_mecanumA' if sy > 0 else 'wheel_mecanumB'
            yaw = -1.5708 if sy > 0 else 1.5708
            out.append(f"""        <visual name="wheel_{fr}{lr}">
          <pose>{x:.4f} {y:.4f} {z:.4f} -1.5708 0 {yaw}</pose>
          <geometry>
            <mesh>
              <uri>model://avular_origin/meshes/{mesh}.obj</uri>
              <scale>{MESH_SCALE} {MESH_SCALE} {MESH_SCALE}</scale>
            </mesh>
          </geometry>
        </visual>""")
    return '\n'.join(out)


def _marker_cells(cube_top):
    grid = marker_grid()
    half = (CELLS - 1) / 2.0
    z = cube_top + 0.0006
    out = []
    for r in range(CELLS):
        for c in range(CELLS):
            if not grid[r][c]:
                continue
            x = (c - half) * CELL
            y = (half - r) * CELL
            out.append(f'''        <visual name="aruco_{r}_{c}">
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
          <geometry><box><size>{CELL:.4f} {CELL:.4f} 0.0004</size></box></geometry>
          <material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>''')
    return '\n'.join(out)


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, '..', 'worlds', 'table_aruco.world')
    with open(os.path.normpath(out_path), 'w') as handle:
        handle.write(build_world())
    print('escrito:', os.path.normpath(out_path))
    print('  robot movil: rectangulo de %.0f x %.0f mm' % (2 * RECT_HALF_X * 1000, 2 * RECT_HALF_Y * 1000))
    print('  cara superior del robot: z = %.4f   centro del cubo en reposo: %.4f'
          % (BODY_TOP, BODY_TOP + CUBE_Z / 2.0))
