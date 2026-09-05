#!/usr/bin/env python3
"""Genera xarm_gazebo/worlds/table_aruco.world (Tarea 2 - visual servoing).

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

CUBE_XY = 0.04     # cubo agarrable: mismo tamano que el small_box de table_gz.world
CUBE_Z = 0.05

TABLE_TOP = 1.00   # altura de la superficie de la mesa de Fuel en este mundo
# Coordenadas del MUNDO. El robot se spawnea en (-0.2, -0.5, 1.021) con yaw
# +90 deg, asi que un punto (bx, by, bz) de link_base cae en el mundo en
# (-0.2 - by, -0.5 + bx, 1.021 + bz). El riel esta puesto sobre la misma zona de
# la mesa que ya usan pick_and_place.py y la Tarea 3: link_base x ~ -0.30,
# y entre -0.28 y -0.52.
RAIL_X = 0.20      # centro del recorrido
RAIL_Y = -0.80
TRAVEL = 0.12      # +- respecto del centro


def marker_grid():
    """Devuelve la matriz CELLSxCELLS del marcador: True = celda negra."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICT)
    img = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, CELLS, borderBits=1)
    return [[img[r][c] == 0 for c in range(CELLS)] for r in range(CELLS)]


def build_world():
    marker_side = CELLS * CELL
    cube_top = CUBE_Z / 2.0
    # Geometria real de la bandeja: el link `cart` esta a +0.0075 del origen del
    # modelo y su piso es una caja de 5 mm centrada ahi, o sea que la cara de
    # arriba del piso queda a +0.010. Calcularlo como TABLE_TOP+0.005 dejaba el
    # cubo 5 mm hundido dentro del piso y el motor lo expulsaba al arrancar.
    cart_floor_top = TABLE_TOP + 0.010
    cube_z = cart_floor_top + CUBE_Z / 2.0

    return f'''<?xml version="1.0" ?>
<!-- GENERADO por xarm_gazebo/scripts/gen_aruco_world.py - no editar a mano. -->
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

    <!-- Carro: una bandeja sobre un riel prismatico. El cubo NO esta pegado al
         carro, solo apoyado dentro: las paredes lo empujan, y por eso se puede
         levantar en vertical sin pelear contra la restriccion del joint. Mover
         el cubo con VelocityControl en vez de esto lo dejaria clavado en el aire
         y el gripper no podria arrancarlo. -->
    <model name="aruco_cart">
      <pose>{RAIL_X} {RAIL_Y} {TABLE_TOP} 0 0 0</pose>
      <static>false</static>

      <link name="rail">
        <visual name="visual">
          <pose>0 0 0.001 0 0 0</pose>
          <geometry>
            <box><size>{2 * TRAVEL + 0.12:.3f} 0.12 0.002</size></box>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.28 1</ambient>
            <diffuse>0.25 0.25 0.28 1</diffuse>
          </material>
        </visual>
      </link>
      <joint name="rail_fixed" type="fixed">
        <parent>world</parent>
        <child>rail</child>
      </joint>

      <link name="cart">
        <pose>0 0 0.0075 0 0 0</pose>
        <inertial>
          <mass>0.5</mass>
          <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
        </inertial>
        <visual name="floor_visual">
          <geometry><box><size>0.10 0.10 0.005</size></box></geometry>
          <material><ambient>0.2 0.35 0.6 1</ambient><diffuse>0.2 0.35 0.6 1</diffuse></material>
        </visual>
        <collision name="floor_collision">
          <geometry><box><size>0.10 0.10 0.005</size></box></geometry>
          <surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode></friction></surface>
        </collision>
{_walls()}
      </link>
      <joint name="slide" type="prismatic">
        <parent>rail</parent>
        <child>cart</child>
        <axis>
          <xyz>1 0 0</xyz>
          <!-- El limite se deja con margen sobre TRAVEL: el controlador de
               posicion sobrepasa un poco al frenar, y llegar al tope mecanico
               en cada extremo mete un golpe que sacude el cubo. -->
          <limit><lower>{-TRAVEL - 0.05:.3f}</lower><upper>{TRAVEL + 0.05:.3f}</upper><effort>200</effort><velocity>1.0</velocity></limit>
          <dynamics><damping>1.0</damping></dynamics>
        </axis>
      </joint>

      <!-- Control de POSICION del riel, no de velocidad. Escucha gz-transport
           en /aruco_cart/cmd_pos (ignition.msgs.Double, posicion del carro en m
           respecto del centro del riel).

           Con el JointController por velocidad la posicion queda a lazo abierto:
           se integra v*dt y el error se acumula, asi que a los pocos ciclos el
           carro llega al tope del joint y ahi se queda clavado -- los comandos
           siguen alternando y el marcador ya no se mueve. Con control de
           posicion cada consigna es absoluta y no hay deriva que acumular. -->
      <plugin filename="ignition-gazebo-joint-position-controller-system"
              name="ignition::gazebo::systems::JointPositionController">
        <joint_name>slide</joint_name>
        <topic>/aruco_cart/cmd_pos</topic>
        <p_gain>200.0</p_gain>
        <i_gain>0.0</i_gain>
        <d_gain>20.0</d_gain>
        <cmd_max>100.0</cmd_max>
        <cmd_min>-100.0</cmd_min>
      </plugin>
    </model>

    <!-- Cubo con el marcador ArUco id {MARKER_ID} ({marker_side * 1000:.0f} mm) en la cara superior. -->
    <model name="aruco_cube">
      <pose>{RAIL_X} {RAIL_Y} {cube_z:.4f} 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <inertial>
          <mass>0.01</mass>
          <inertia><ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-5</iyy><iyz>0</iyz><izz>1e-5</izz></inertia>
        </inertial>
        <visual name="body">
          <geometry><box><size>{CUBE_XY} {CUBE_XY} {CUBE_Z}</size></box></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
        <!-- Placa blanca: el borde claro que el detector necesita alrededor del patron. -->
        <visual name="marker_plate">
          <pose>0 0 {cube_top + 0.0002:.4f} 0 0 0</pose>
          <geometry><box><size>{marker_side + 2 * CELL:.4f} {marker_side + 2 * CELL:.4f} 0.0004</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
        <!-- Celdas negras del patron, elevadas 0.4 mm sobre la placa blanca para
             que no haya z-fighting con ella. -->
{_marker_cells(cube_top)}
        <!-- Estos parametros de superficie son los mismos, al pie de la letra,
             que los del small_box de table_gz.world, que es el cubo que agarra
             pick_and_place.py. No son "realistas": mu=1000 y friccion torsional
             son un truco para que un gripper paralelo controlado por POSICION
             pueda sostener un objeto libre en Gazebo. Con mu=2 (un valor
             fisicamente sensato) el cubo se resbala de los dedos y el brazo
             levanta el aire, aunque el agarre se vea perfecto en camara. -->
        <collision name="collision">
          <geometry><box><size>{CUBE_XY} {CUBE_XY} {CUBE_Z}</size></box></geometry>
          <surface>
            <friction>
              <ode>
                <mu>1000.0</mu>
                <mu2>1000.0</mu2>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
                <fdir1>0 0 1</fdir1>
              </ode>
              <torsional>
                <coefficient>100.0</coefficient>
                <use_patch_radius>true</use_patch_radius>
                <patch_radius>0.1</patch_radius>
                <surface_radius>0.05</surface_radius>
              </torsional>
            </friction>
            <contact>
              <ode>
                <kp>1e8</kp>
                <kd>1000</kd>
                <max_vel>0.0</max_vel>
                <min_depth>0.002</min_depth>
              </ode>
            </contact>
            <bounce>
              <restitution_coefficient>0</restitution_coefficient>
              <threshold>0</threshold>
            </bounce>
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


def _walls():
    """Cuatro paredes bajas: empujan el cubo sin estorbar al gripper."""
    h, t, inner = 0.015, 0.005, 0.10
    z = 0.0025 + h / 2.0
    specs = [
        ('wall_xp', f'{inner / 2 - t / 2:.4f} 0 {z:.4f}', f'{t} {inner} {h}'),
        ('wall_xn', f'{-(inner / 2 - t / 2):.4f} 0 {z:.4f}', f'{t} {inner} {h}'),
        ('wall_yp', f'0 {inner / 2 - t / 2:.4f} {z:.4f}', f'{inner} {t} {h}'),
        ('wall_yn', f'0 {-(inner / 2 - t / 2):.4f} {z:.4f}', f'{inner} {t} {h}'),
    ]
    out = []
    for name, pose, size in specs:
        out.append(f'''        <visual name="{name}_visual">
          <pose>{pose} 0 0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>0.2 0.35 0.6 1</ambient><diffuse>0.2 0.35 0.6 1</diffuse></material>
        </visual>
        <collision name="{name}_collision">
          <pose>{pose} 0 0 0</pose>
          <geometry><box><size>{size}</size></box></geometry>
        </collision>''')
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
