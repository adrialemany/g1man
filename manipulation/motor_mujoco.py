import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json
import time
import threading

# Mapeo de nombres a IDs para el escudo de filtrado
FULL_JOINT_MAP = {
    "left_shoulder_pitch_joint": 15, "left_shoulder_roll_joint": 16, "left_shoulder_yaw_joint": 17, 
    "left_elbow_joint": 18, "left_wrist_roll_joint": 19, "left_wrist_pitch_joint": 20, "left_wrist_yaw_joint": 21,
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23, "right_shoulder_yaw_joint": 24, 
    "right_elbow_joint": 25, "right_wrist_roll_joint": 26, "right_wrist_pitch_joint": 27, "right_wrist_yaw_joint": 28
}

class G1BrazosForzados(Node):
    def __init__(self):
        super().__init__('g1_brazos_forzados')
        
        # Suscriptor para conocer la posición inicial si fuera necesario
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ia_addr = ('127.0.0.1', 9876)
        
        self.state_received = False
        self.dt = 0.02

        # DEFINICIÓN CLARA DE ÁNGULOS (Modifica aquí los valores en radianes)
        self.targets = {
            # IZQUIERDO
            15: 0.6, 16: 0.3, 17: 0.9, 18: -1.5, 19: -1.8, 20: -0.2, 21: 1.0,
            # DERECHO
            22: 0.6, 23: -0.3, 24: -0.9, 25: -1.5, 26: 1.8, 27: -0.2, 28: -1.0
        }
        
        # Hilo de publicación constante para que nadie mueva los brazos excepto nosotros
        threading.Thread(target=self.udp_publisher_loop, daemon=True).start()

    def udp_publisher_loop(self):
        while rclpy.ok():
            if self.targets:
                # Solo enviamos los índices de los brazos; la pelvis (12-14) queda para la IA
                payload = {str(k): float(v) for k, v in self.targets.items()}
                self.udp_sock.sendto(json.dumps(payload).encode(), self.ia_addr)
            time.sleep(0.02)

    def state_callback(self, msg):
        if len(msg.name) >= 29:
            self.state_received = True

    def stop(self):
        print("\n[INFO] Soltando brazos. Devolviendo control total a la IA...")
        self.targets = {}
        self.udp_sock.sendto(json.dumps({}).encode(), self.ia_addr)

def main(args=None):
    rclpy.init(args=args)
    nodo = G1BrazosForzados()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    spin_thread.start()

    print("\n" + "="*60)
    print("🚀 FORZANDO POSTURA DE BRAZOS (Pelvis libre para la IA)")
    print("Postura: Codos atrás, brazos plegados, palmas afuera.")
    print("Presiona Ctrl+C para detener y soltar los brazos.")
    print("="*60)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        nodo.stop()
        time.sleep(0.5)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
