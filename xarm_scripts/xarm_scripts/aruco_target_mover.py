#!/usr/bin/env python3
"""Conduce el robot movil que transporta el marcador ArUco (Tarea 2).

El robot da UNA vuelta a un rectangulo sobre la mesa y, al completarla, **se
estaciona para siempre** en el centro. Esa es la estructura de la tarea:

    mientras se mueve   -> el alumno lo sigue con la camara (Ejercicio 2)
    cuando se detiene   -> el alumno baja y le saca el cubo (Ejercicio 3)

Da la vuelta DE CORRIDO: las esquinas van redondeadas y el robot no frena en
ellas. Antes frenaba hasta 0 en cada vertice, y eso obligaba al alumno a
distinguir "paro en una esquina" de "se estaciono", que es un problema aparte y
mas dificil que el lazo visual. Ahora la unica velocidad cero de toda la corrida
es el estacionamiento, y detectarlo es un umbral y nada mas.

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
                   Por defecto va 1: a 0.03 m/s (`slow`, que es lo que pide el
                   enunciado) una vuelta dura unos 23 s, y el Ejercicio 2 pide
                   sostener el seguimiento 10 s, asi que sobra de largo.
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

# Radio con el que se redondean las cuatro esquinas. Recorrer un arco a
# velocidad constante cuesta una aceleracion centripeta de v^2/r, que tiene que
# quedar por debajo de ACCEL para que el cubo no patine sobre el chasis: con
# r = 0.03 son 0.03 m/s^2 en `slow` y 0.12 en `medium`, los dos por debajo de
# 0.15. Tomar el vertice en angulo recto sin frenar seria un cambio de velocidad
# instantaneo, o sea aceleracion infinita, y el cubo saldria volando.
CORNER_RADIUS = 0.03

# Frenar se hace con la mitad de ACCEL. El techo sqrt(2*a*d) da exactamente `a`
# en continuo, pero a 50 Hz el ultimo tramo se pasa un poco; con margen, el
# tiron real se queda por debajo del limite.
DECEL = 0.5 * ACCEL

# Semiejes por defecto: tienen que coincidir con RECT_HALF_X / RECT_HALF_Y de
# xarm_gazebo/scripts/gen_aruco_world.py, que es de donde salen los topes de las
# juntas en el .world.
DEFAULT_HALF_X = 0.11
DEFAULT_HALF_Y = 0.08


class ArucoTargetMover(Node):

    def __init__(self):
        super().__init__('aruco_target_mover')

        self.declare_parameter('mode', 'slow')
        self.declare_parameter('speed', -1.0)
        self.declare_parameter('laps', 1)
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

        # El recorrido es un rectangulo de esquinas redondeadas, precalculado
        # como una lista de tramos (rectas y arcos) con su longitud. Se avanza
        # sobre la distancia recorrida `s`, no de waypoint en waypoint: asi la
        # velocidad no tiene que bajar a cero para cambiar de direccion.
        self.track = self.build_track(hx, hy, CORNER_RADIUS)
        self.track_len = sum(seg['len'] for seg in self.track)

        self.s = 0.0           # distancia recorrida en la vuelta actual
        self.pos = [0.0, 0.0]  # el robot aparece en el centro de la mesa
        self.lap = 0
        self.current_speed = 0.0
        self.starting = True   # yendo del centro al inicio del recorrido
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

    @staticmethod
    def build_track(hx, hy, r):
        """Rectangulo de esquinas redondeadas, como lista de tramos.

        Cada esquina de 90 grados se sustituye por un cuarto de circunferencia
        de radio `r` tangente a los dos lados. Los puntos de tangencia quedan a
        distancia `r` del vertice sobre cada lado.
        """
        corners = [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)]

        def unit(a, b):
            dx, dy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(dx, dy)
            return (dx / n, dy / n)

        arcs = []
        for i, c in enumerate(corners):
            u = unit(corners[i - 1], c)                    # direccion de entrada
            w = unit(c, corners[(i + 1) % 4])              # direccion de salida
            t_in = (c[0] - r * u[0], c[1] - r * u[1])
            t_out = (c[0] + r * w[0], c[1] + r * w[1])
            # El centro esta a distancia r del punto de tangencia, en la
            # direccion hacia la que se dobla.
            centre = (t_in[0] + r * w[0], t_in[1] + r * w[1])
            a0 = math.atan2(t_in[1] - centre[1], t_in[0] - centre[0])
            a1 = math.atan2(t_out[1] - centre[1], t_out[0] - centre[0])
            # El recorrido es horario: el angulo siempre decrece 90 grados.
            while a1 > a0:
                a1 -= 2.0 * math.pi
            arcs.append({'type': 'arc', 'centre': centre, 'r': r,
                         'a0': a0, 'a1': a1, 'in': t_in, 'out': t_out,
                         'len': r * abs(a1 - a0)})

        track = []
        for i, arc in enumerate(arcs):
            nxt = arcs[(i + 1) % 4]
            p0, p1 = arc['out'], nxt['in']
            track.append({'type': 'line', 'p0': p0, 'p1': p1,
                          'len': math.dist(p0, p1)})
            track.append(nxt)
        return track

    def point_at(self, s):
        """Punto del recorrido a distancia `s` del inicio."""
        s = s % self.track_len
        for seg in self.track:
            if s <= seg['len'] or seg is self.track[-1]:
                if seg['type'] == 'line':
                    f = s / seg['len'] if seg['len'] else 0.0
                    return (seg['p0'][0] + (seg['p1'][0] - seg['p0'][0]) * f,
                            seg['p0'][1] + (seg['p1'][1] - seg['p0'][1]) * f)
                f = s / seg['len'] if seg['len'] else 0.0
                a = seg['a0'] + (seg['a1'] - seg['a0']) * f
                return (seg['centre'][0] + seg['r'] * math.cos(a),
                        seg['centre'][1] + seg['r'] * math.sin(a))
            s -= seg['len']
        return self.pos

    def advance(self):
        if self.parking:
            if self.drive_towards(0.0, 0.0):
                self.parked = True
                self.get_logger().info(
                    'recorrido terminado: el robot queda estacionado. '
                    'Es el momento de bajar y agarrar el cubo.')
            return

        if self.starting:
            # El robot aparece en el centro, pero el recorrido empieza en un
            # costado: hay que LLEVARLO hasta ahi con la misma rampa. Poner la
            # consigna directamente en el inicio es un salto de 121 mm en un
            # solo tick, y el cubo, que va sujeto solo por friccion, patina.
            if self.drive_towards(*self.point_at(0.0)):
                self.starting = False
            return

        # Vuelta de corrido: se acelera hasta la velocidad de crucero y ya no se
        # baja. No hay frenada en las esquinas, van redondeadas.
        self.current_speed = min(self.speed,
                                 self.current_speed + ACCEL * PUBLISH_PERIOD)

        # En la ultima vuelta se va frenando sobre el propio recorrido, para
        # llegar al final ya casi parado. Sin esto, encarar el centro a
        # velocidad de crucero es un cambio de direccion instantaneo: un tiron
        # de 1.6 m/s^2, diez veces el limite con el que el cubo no patina.
        if self.laps > 0 and self.lap == self.laps - 1:
            falta = max(self.track_len - self.s, 0.0)
            self.current_speed = min(self.current_speed,
                                     math.sqrt(2.0 * DECEL * falta))

        self.s += self.current_speed * PUBLISH_PERIOD

        if self.s >= self.track_len:
            self.s -= self.track_len
            self.lap += 1
            self.get_logger().info(f'vuelta {self.lap} completada')
            if self.laps > 0 and self.lap >= self.laps:
                # No se salta al centro de golpe: se va conduciendo hasta ahi, si
                # no la consigna pega un salto y el cubo sale despedido.
                self.parking = True

        self.pos = list(self.point_at(self.s))

    def drive_towards(self, tx, ty):
        """Va en linea recta hasta (tx, ty) y frena al llegar. True si llego.

        Se usa en los dos tramos que no son la vuelta: la ida del centro al
        inicio del recorrido y la vuelta al centro para estacionar. Los dos
        arrancan y terminan parados, asi que aca la rampa si hace falta.
        """
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-5:
            self.pos = [tx, ty]
            self.current_speed = 0.0
            return True

        # Techo de velocidad con el que todavia se puede frenar a tiempo:
        # v = sqrt(2*a*d). Poniendolo como tope, la rampa baja sola hasta cero
        # justo al llegar. Con un if de "ya toca frenar" el ultimo paso queda
        # corto y la velocidad cae de golpe: un tiron de mas de 1 m/s^2 que
        # basta para mover el cubo sobre el chasis.
        self.current_speed = min(self.speed,
                                 self.current_speed + ACCEL * PUBLISH_PERIOD,
                                 math.sqrt(2.0 * DECEL * dist))

        step = min(self.current_speed * PUBLISH_PERIOD, dist)
        self.pos[0] += dx / dist * step
        self.pos[1] += dy / dist * step
        return False


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
