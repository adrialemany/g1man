#!/usr/bin/env python3
"""
cmd_vel_bridge.py
=================
Puente genérico entre cualquier navegador ROS 2 y el simulador G1.

Funciona idéntico con Nav2 o con EasyNavigation: ambos publican Twist
en /cmd_vel, y este puente lo traduce a comandos TCP sobre el mismo
puerto 6000 que ya escucha run_sim_ai_g1.py.

Suscribe:
    /cmd_vel  (geometry_msgs/Twist)  — velocidades del controller activo

Acción:
    Convierte cada Twist en comandos TCP enviados al puerto 6000 de
    run_sim_ai_g1.py (mismo protocolo que el cliente Tkinter):
        'w'/'s' = avance/retroceso
        'a'/'d' = strafe izquierda/derecha
        'q'/'e' = rotación izquierda/derecha
        'stop'  = parar

    Usa una política de "componente dominante": en cada ciclo elige el eje
    con mayor magnitud (lin.x, lin.y o ang.z) y manda la tecla correspondiente.
    Esto reaprovecha exactamente el sistema de teleop existente, sin tocar
    run_sim_ai_g1.py.

Uso:
    python3 cmd_vel_bridge.py

    # Asegúrate de tener arrancado:
    #   - run_sim_ai_g1.py (escucha en TCP 6000)
    #   - mujoco_ros2_lidar_bridge.py
    #   - El navegador activo: Nav2 o EasyNavigation
"""

import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist


# ===========================================================================
# Configuración
# ===========================================================================
TCP_HOST = "127.0.0.1"
TCP_PORT = 6000

# Umbral mínimo: por debajo de esto se considera "parado"
DEAD_ZONE_LIN = 0.05    # m/s
DEAD_ZONE_ANG = 0.10    # rad/s

# Frecuencia máxima de envío TCP (limita la carga)
MAX_TCP_HZ = 20.0

# Topic
CMD_VEL_TOPIC = "/cmd_vel"


class Nav2CmdVelBridge(Node):
    # Nombre histórico; sirve también para EasyNav (ambos publican /cmd_vel)

    def __init__(self):
        super().__init__("cmd_vel_bridge")

        # Nav2 publica /cmd_vel con QoS por defecto (RELIABLE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Twist, CMD_VEL_TOPIC, self._cmd_cb, qos)

        self._last_send_time = 0.0
        self._last_cmd = None

        self.get_logger().info(
            f"Nav2CmdVelBridge arrancado.\n"
            f"  Suscrito: {CMD_VEL_TOPIC}\n"
            f"  Reenvía a TCP {TCP_HOST}:{TCP_PORT}\n"
            f"  Dead zone: {DEAD_ZONE_LIN} m/s, {DEAD_ZONE_ANG} rad/s"
        )

    # -------------------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        """Convierte un Twist en una tecla y la envía por TCP."""
        # Rate limiting
        now = time.time()
        if now - self._last_send_time < 1.0 / MAX_TCP_HZ:
            return

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Decidir comando dominante
        cmd = self._choose_command(vx, vy, wz)

        # Solo enviar si cambia (para no inundar el TCP)
        if cmd != self._last_cmd:
            self._send_tcp(cmd)
            self._last_cmd = cmd
            self.get_logger().debug(
                f"vx={vx:.2f} vy={vy:.2f} wz={wz:.2f} → {cmd}")

        self._last_send_time = now

    # -------------------------------------------------------------------
    @staticmethod
    def _choose_command(vx: float, vy: float, wz: float) -> str:
        """
        Elige la tecla dominante según las magnitudes de cada eje.
        Devuelve 'w','s','a','d','q','e' o 'stop'.
        """
        abs_vx = abs(vx)
        abs_vy = abs(vy)
        abs_wz = abs(wz)

        # Dead zone: si todo es pequeño, parar
        if (abs_vx < DEAD_ZONE_LIN and
            abs_vy < DEAD_ZONE_LIN and
            abs_wz < DEAD_ZONE_ANG):
            return 'stop'

        # Eje dominante (normalizamos rad/s vs m/s con un factor)
        # Para que rotaciones de 0.5 rad/s puedan competir con 0.3 m/s
        score_lin_x = abs_vx
        score_lin_y = abs_vy
        score_ang   = abs_wz * 0.5  # peso angular

        max_score = max(score_lin_x, score_lin_y, score_ang)

        if max_score == score_lin_x:
            return 'w' if vx > 0 else 's'
        elif max_score == score_lin_y:
            # IMPORTANTE: en run_sim_ai_g1.py 'a' pone vy=+0.3 (izquierda)
            return 'a' if vy > 0 else 'd'
        else:
            # 'q' pone yaw=+0.6 (rotación izquierda)
            return 'q' if wz > 0 else 'e'

    # -------------------------------------------------------------------
    def _send_tcp(self, cmd: str):
        """Envía un comando al puerto 6000 de run_sim_ai_g1.py."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.3)
                s.connect((TCP_HOST, TCP_PORT))
                s.sendall(cmd.encode('utf-8'))
                try:
                    s.recv(128)
                except socket.timeout:
                    pass
        except Exception as e:
            self.get_logger().warn(f"No se pudo enviar TCP: {e}")


# ===========================================================================
# main
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Asegurar que el robot se detiene al cerrar
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.3)
                s.connect((TCP_HOST, TCP_PORT))
                s.sendall(b'stop')
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
