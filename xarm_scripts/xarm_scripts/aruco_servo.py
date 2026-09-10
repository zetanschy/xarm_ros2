#!/usr/bin/env python3
"""Ejercicio 1: visual servoing con MoveIt Servo.

Completa los tres TODO marcados mas abajo y recompila. El ejecutable ya esta
declarado en el setup.py de xarm_scripts, asi que se lanza tal cual:

    ros2 launch xarm_scripts visual_servo.launch.py mode:=slow node:=aruco_servo

La plomeria (deteccion del marcador, arranque del servo, gripper, TF, secuencia
de agarre) esta en servo_utils.py y no hay que tocarla. Lo que se usa de ahi:

    self.cam.ready()      True cuando llegaron los intrinsecos de la camara
    self.cam.visible()    True si se ve el marcador ahora mismo
    self.cam.centre()     (u, v) del centro del marcador, en pixeles
    self.cam.fx           distancia focal en pixeles
    self.cam.cx, .cy      centro de la imagen, en pixeles

    self.arm.move(vx, vy, vz)   velocidad en m/s (los tres son opcionales)
    self.arm.stop()             frena el brazo
    self.arm.descend_speed(e)   cuanto bajar, dado el error de centrado
    self.arm.at_grasp_height()  True si ya esta a la altura del cubo
    self.arm.grab_and_lift()    cierra y sube; vuelve enseguida
    self.arm.busy               True mientras esa secuencia corre
    self.arm.done               True cuando ya levanto el cubo: se acabo

Por debajo, `arm.move()` publica un geometry_msgs/TwistStamped en
/servo_node/delta_twist_cmds: eso es todo lo que recibe MoveIt Servo. El servo
lo convierte en velocidades de junta con la jacobiana y las manda al
controlador. Los twists van en el frame optico de la camara:

    +x = derecha de la imagen    +y = abajo de la imagen    +z = hacia la mesa

y la camara es coaxial con el TCP, asi que centrar el marcador en la imagen es
lo mismo que poner el gripper encima del cubo. No hay que transformar nada.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from xarm_scripts.servo_utils import MarkerCamera, ServoArm

KP_XY = 0.1              # TODO 1: este valor NO alcanza. Ver el enunciado.
CENTER_TOL_PX = 45.0     # error con el que ya se puede empezar a bajar
GRASP_TOL_PX = 18.0      # error con el que ya se puede cerrar el gripper
STILL_SPEED = 0.012      # m/s por debajo de los cuales el carro esta parado
STILL_TIME = 1.0         # s que esa quietud tiene que sostenerse


class ArucoServo(Node):

    def __init__(self):
        super().__init__('aruco_servo')
        self.cam = MarkerCamera(self)
        self.arm = ServoArm(self)
        self.error_pub = self.create_publisher(Float32, '/tracking_error', 10)

        self.descending = False
        self.still_since = None

        # 50 Hz pase lo que pase. Publicar dentro del callback de la imagen
        # (30 Hz y con jitter) hace que el brazo vaya a tirones.
        self.create_timer(0.02, self.control)

    def control(self):
        # done = ya levanto el cubo. Sin esto el lazo seguiria corriendo con el
        # cubo delante de la camara, y el brazo volveria a bajar a "agarrarlo".
        if self.arm.done or self.arm.busy or not self.cam.ready():
            return

        if not self.cam.visible():
            # TODO 2 - COMPLETAR: frena el brazo. Dejar de mandar comandos no
            # lo frena, lo deja repitiendo el ultimo. Y pone self.still_since
            # en None, o un parpadeo de la deteccion te deja el reloj corriendo.
            return

        u, v = self.cam.centre()

        # TODO 1 - Completa estas cinco lineas.
        ex = "COMPLETAR"      # cuanto se desvio el marcador del centro, en x (px)
        ey = "COMPLETAR"      # lo mismo en y (px)
        err_px = "COMPLETAR"  # distancia al centro: math.hypot() de las dos de arriba
        vx = "COMPLETAR"      # ex --> dividir por self.cam.fx --> por KP_XY --> m/s
        vy = "COMPLETAR"      # lo mismo con ey

        self.error_pub.publish(Float32(data=float(err_px)))

        if not self.descending:
            self.arm.move(vx, vy)
            if err_px < CENTER_TOL_PX:
                self.descending = True
                self.get_logger().info(f'centrado ({err_px:.0f} px): a bajar')
            return

        # TODO 3 - COMPLETAR: ¿el carro se detuvo? Sin temporizador. La
        # velocidad del marcador EN LA IMAGEN no sirve: llevarla a cero es lo
        # que hace tu lazo. Hay otra magnitud, ya calculada arriba, que vale lo
        # mismo. Comparala con STILL_SPEED.
        still = "COMPLETAR"

        # Cronometro: arranca cuando el carro se queda quieto y se reinicia si
        # se vuelve a mover. Solo se da por estacionado si aguanta STILL_TIME.
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = time.time()

        parked = still and time.time() - self.still_since > STILL_TIME

        if parked:
            vz = self.arm.descend_speed(err_px)
        else:
            vz = 0.0        # el carro se mueve: se le sigue, pero sin bajar
        self.arm.move(vx, vy, vz)

        if parked and err_px < GRASP_TOL_PX and self.arm.at_grasp_height():
            self.arm.grab_and_lift()


def main():
    rclpy.init()
    node = ArucoServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
