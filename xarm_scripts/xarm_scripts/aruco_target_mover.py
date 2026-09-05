#!/usr/bin/env python3
"""Mueve el carro del marcador ArUco en table_aruco.world (Tarea 2).

Publica la POSICION objetivo del joint prismatico `slide` del modelo
`aruco_cart`. El puente ros_ign la traduce a gz-transport y el
JointPositionController del mundo la sigue.

Se comanda posicion y no velocidad a proposito: con velocidad, la posicion del
carro queda a lazo abierto (se integra v*dt), el error se acumula ciclo a ciclo
y a los pocos minutos el carro termina clavado contra el tope del joint, con el
marcador quieto y los comandos alternando sin efecto. Con posicion, cada
consigna es absoluta.

El recorrido es una rampa triangular con pausas en los extremos. Las pausas son
la ventana comoda para cerrar el gripper: agarrar un objeto que se mueve con un
brazo controlado por posicion es duro, y la idea de la tarea es el lazo visual,
no la punteria.

Parametros:
  mode    (str)         static | slow | medium | fast  (fija speed y pause)
  speed   (float, m/s)  velocidad del carro; <0 = usar el preset de `mode`
  pause   (float, s)    cuanto se detiene en cada extremo; <0 = preset
  travel  (float, m)    semi-recorrido. Tiene que ser <= la constante TRAVEL de
                        xarm_gazebo/scripts/gen_aruco_world.py, que es de donde
                        sale el limite del joint en el .world.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# (velocidad m/s, pausa en cada extremo s)
PRESETS = {
    'static': (0.00, 0.0),
    'slow': (0.03, 3.0),
    'medium': (0.06, 2.0),
    'fast': (0.12, 1.0),
}

PUBLISH_PERIOD = 0.02  # 50 Hz: la rampa tiene que ser fina o el carro da saltos


class ArucoTargetMover(Node):

    def __init__(self):
        super().__init__('aruco_target_mover')

        self.declare_parameter('mode', 'slow')
        self.declare_parameter('speed', -1.0)
        self.declare_parameter('pause', -1.0)
        self.declare_parameter('travel', 0.12)

        mode = self.get_parameter('mode').value
        if mode not in PRESETS:
            self.get_logger().warn(
                f"mode '{mode}' desconocido; se usa 'slow'. "
                f"Opciones: {', '.join(PRESETS)}")
            mode = 'slow'
        preset_speed, preset_pause = PRESETS[mode]

        speed = self.get_parameter('speed').value
        pause = self.get_parameter('pause').value
        self.speed = preset_speed if speed < 0 else speed
        self.pause = preset_pause if pause < 0 else pause
        self.travel = self.get_parameter('travel').value

        self.pub = self.create_publisher(Float64, '/aruco_cart/cmd_pos', 10)

        self.position = 0.0        # consigna actual, respecto del centro del riel
        self.direction = 1.0
        self.pause_left = 0.0

        self.create_timer(PUBLISH_PERIOD, self.tick)
        self.get_logger().info(
            f'mode={mode} speed={self.speed:.3f} m/s pause={self.pause:.1f} s '
            f'travel=+-{self.travel:.2f} m')
        if self.speed <= 0.0:
            self.get_logger().info('velocidad 0: el marcador se queda quieto')

    def tick(self):
        if self.speed > 0.0:
            if self.pause_left > 0.0:
                self.pause_left -= PUBLISH_PERIOD
            else:
                self.position += self.direction * self.speed * PUBLISH_PERIOD
                if abs(self.position) >= self.travel:
                    # Se recorta a proposito en vez de dejarlo pasar: asi la
                    # consigna nunca supera el recorrido, aunque el paso de
                    # integracion no caiga justo en el extremo.
                    self.position = self.direction * self.travel
                    self.direction *= -1.0
                    self.pause_left = self.pause
        self.pub.publish(Float64(data=float(self.position)))

    def stop(self):
        """Deja la consigna en el centro antes de salir."""
        for _ in range(5):
            self.pub.publish(Float64(data=0.0))


def main():
    rclpy.init()
    node = ArucoTargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
