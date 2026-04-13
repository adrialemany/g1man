"""
mirror_arm.py – Espejo cinemático para el Unitree G1.
Escucha la posición del brazo derecho y obliga al brazo izquierdo a imitarlo simétricamente.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json
import time
import threading

# Mapeo de articulaciones (Derecha -> Izquierda)
# y factor de multiplicación (algunos motores giran al revés en el otro brazo)
MIRROR_MAP = {
    # right_joint_name : (left_motor_id, multiplier)
    "right_shoulder_pitch_joint": (15, 1.0),   # Mismo sentido (hacia adelante/atrás)
    "right_shoulder_roll_joint":  (16, -1.0),  # Sentido inverso (hacia afuera/adentro)
    "right_shoulder_yaw_joint":   (17, -1.0),  # Sentido inverso
    "right_elbow_joint":          (18, 1.0),   # Mismo sentido (doblar)
    "right_wrist_roll_joint":     (19, -1.0),  # Sentido inverso
    "right_wrist_pitch_joint":    (20, 1.0),   # Mismo sentido
    "right_wrist_yaw_joint":      (21, -1.0),  # Sentido inverso
}

class MirrorArm(Node):
    def __init__(self):
        super().__init__('mirror_arm')
        
        # Suscribirse al estado real de los motores (que MoveIt está controlando)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        
        # Socket UDP para inyectar los comandos del brazo izquierdo al run_sim_ai_g1
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ia_addr = ('127.0.0.1', 9876)
        
        self.get_logger().info("🪞 Script Espejo INICIADO: Brazo Izquierdo imitará al Derecho.")
        self.last_send_time = time.time()

    def joint_callback(self, msg):
        """ Se ejecuta cada vez que hay una actualización de posiciones """
        now = time.time()
        # Limitar envío a ~30Hz para no saturar MuJoCo
        if now - self.last_send_time < 0.03: 
            return
            
        payload = {}
        for i, name in enumerate(msg.name):
            if name in MIRROR_MAP:
                left_motor_id, multiplier = MIRROR_MAP[name]
                right_angle = msg.position[i]
                
                # Calculamos el ángulo espejo
                left_angle = right_angle * multiplier
                payload[str(left_motor_id)] = left_angle
        
        if payload:
            self.udp_sock.sendto(json.dumps(payload).encode(), self.ia_addr)
            self.last_send_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MirrorArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
