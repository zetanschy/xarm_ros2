#!/usr/bin/env python3
"""Conduce el robot movil que transporta el marcador ArUco (Tarea 2).

El robot recorre un rectangulo sobre la mesa y, al completarlo, **se estaciona
para siempre** en el centro. Esa es la estructura de la tarea:

    mientras se mueve   -> el alumno lo sigue con la camara (Ejercicio 2)
    cuando se detiene   -> el alumno baja y le saca el cubo (Ejercicio 3)

Se estaciona de verdad, no hace una pausa: agarrar un objeto en movimiento con un
brazo controlado por posicion y una camara de 30 Hz es un problema bastante mas
duro que el que plantea esta tarea, y el objetivo aca es el lazo visual, no la
punteria.

Publica las consignas de POSICION de las dos juntas prismaticas del robot. Se
comanda posicion y no velocidad porque con velocidad la posicion queda a lazo
abierto: se integra v*dt, el error se acumula, y el robot termina clavado contra
el tope de una junta con el marcador quieto y los comandos alternando sin efecto.

Parametros:
  mode    (str)    static | slow | medium | fast   (fija la velocidad)
  speed   (float)  m/s; <0 = usar el preset de `mode`
  laps    (int)    vueltas al rectangulo antes de estacionar (0 = no parar nunca).
                   Una vuelta a 0.06 m/s dura unos 11 s; el Ejercicio 2 pide
                   sostener el seguimiento 10 s, asi que con una sola vuelta el
                   alumno no llega. Por defecto van 3.
  half_x  (float)  semieje del rectangulo en X, en metros
  half_y  (float)  semieje del rectangulo en Y
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

SPEEDS = {'static': 0.0, 'slow': 0.03, 'medium': 0.06, 'fast': 0.12}

PUBLISH_PERIOD = 0.02  # 50 Hz: la rampa tiene que ser fina o el robot da saltos

# El cubo viaja apoyado sobre el robot, sujeto solo por friccion. Arrancar y
# frenar de golpe lo hace patinar y, a la larga, caerse. Con 0.15 m/s^2 tarda
# menos de medio segundo en llegar a velocidad de crucero y no se mueve respecto
# del chasis.
ACCEL = 0.15  # m/s^2

# Semiejes por defecto: tienen que coincidir con RECT_HALF_X / RECT_HALF_Y de
# xarm_gazebo/scripts/gen_aruco_world.py, que es de donde salen los topes de las
# juntas en el .world.
DEFAULT_HALF_X = 0.08
DEFAULT_HALF_Y = 0.06


class ArucoTargetMover(Node):

    def __init__(self):
        super().__init__('aruco_target_mover')

        self.declare_parameter('mode', 'slow')
        self.declare_parameter('speed', -1.0)
        self.declare_parameter('laps', 3)
        self.declare_parameter('half_x', DEFAULT_HALF_X)
        self.declare_parameter('half_y', DEFAULT_HALF_Y)

        mode = self.get_parameter('mode').value
        if mode not in SPEEDS:
            self.get_logger().warn(
                f"mode '{mode}' desconocido; se usa 'slow'. "
                f"Opciones: {', '.join(SPEEDS)}")
            mode = 'slow'

        speed = self.get_parameter('speed').value
        self.speed = SPEEDS[mode] if speed < 0 else speed
        self.laps = int(self.get_parameter('laps').value)
        hx = self.get_parameter('half_x').value
        hy = self.get_parameter('half_y').value

        self.pub_x = self.create_publisher(Float64, '/mobile_robot/cmd_x', 10)
        self.pub_y = self.create_publisher(Float64, '/mobile_robot/cmd_y', 10)

        # Las cuatro esquinas del rectangulo, en orden. Al terminar las vueltas
        # el robot va al centro y se estaciona ahi, que es donde el brazo lo
        # alcanza mas comodo.
        self.corners = [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)]

        self.pos = [0.0, 0.0]
        self.idx = 0
        self.lap = 0
        self.current_speed = 0.0
        self.parking = False   # yendo al centro
        self.parked = False    # ya llego y no se mueve mas

        self.create_timer(PUBLISH_PERIOD, self.tick)
        self.get_logger().info(
            f'mode={mode} speed={self.speed:.3f} m/s laps={self.laps} '
            f'rectangulo={2 * hx:.2f} x {2 * hy:.2f} m')
        if self.speed <= 0.0:
            self.get_logger().info('velocidad 0: el robot no se mueve')

    def tick(self):
        if not self.parked and self.speed > 0.0:
            self.advance()
        msg_x = Float64(data=float(self.pos[0]))
        msg_y = Float64(data=float(self.pos[1]))
        self.pub_x.publish(msg_x)
        self.pub_y.publish(msg_y)

    def advance(self):
        tx, ty = (0.0, 0.0) if self.parking else self.corners[self.idx % 4]
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        dist = math.hypot(dx, dy)

        if dist < 1e-4:
            self.next_waypoint()
            return

        # Frenar al acercarse a la esquina: la distancia necesaria es v^2 / (2a).
        # Sin esto el robot llega a cada esquina a velocidad plena, se detiene de
        # golpe, y el cubo sale despedido hacia adelante.
        if dist <= self.current_speed ** 2 / (2.0 * ACCEL):
            self.current_speed = max(0.0, self.current_speed - ACCEL * PUBLISH_PERIOD)
        else:
            self.current_speed = min(self.speed, self.current_speed + ACCEL * PUBLISH_PERIOD)

        step = self.current_speed * PUBLISH_PERIOD
        if step >= dist or self.current_speed <= 0.0:
            self.pos = [tx, ty]
            self.next_waypoint()
        else:
            self.pos[0] += dx / dist * step
            self.pos[1] += dy / dist * step

    def next_waypoint(self):
        self.current_speed = 0.0
        if self.parking:
            # Llego al centro: se apaga. A partir de aca el marcador esta quieto
            # y el alumno puede bajar a agarrar el cubo.
            self.parked = True
            self.get_logger().info(
                'recorrido terminado: el robot queda estacionado. '
                'Es el momento de bajar y agarrar el cubo.')
            return

        self.idx += 1
        if self.idx % 4 == 0:
            self.lap += 1
            self.get_logger().info(f'vuelta {self.lap} completada')
            if self.laps > 0 and self.lap >= self.laps:
                # No se salta al centro de golpe: se va conduciendo hasta ahi, si
                # no la consigna pega un salto y el cubo sale despedido.
                self.parking = True


def main():
    rclpy.init()
    node = ArucoTargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
