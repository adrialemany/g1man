import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json
import time
import threading

# Diccionario maestro para mapear cualquier nombre a su ID en MuJoCo
FULL_JOINT_MAP = {
    "left_hip_pitch_joint": 0, "left_hip_roll_joint": 1, "left_hip_yaw_joint": 2, "left_knee_joint": 3, "left_ankle_pitch_joint": 4, "left_ankle_roll_joint": 5,
    "right_hip_pitch_joint": 6, "right_hip_roll_joint": 7, "right_hip_yaw_joint": 8, "right_knee_joint": 9, "right_ankle_pitch_joint": 10, "right_ankle_roll_joint": 11,
    "waist_yaw_joint": 12, "waist_roll_joint": 13, "waist_pitch_joint": 14,
    "left_shoulder_pitch_joint": 15, "left_shoulder_roll_joint": 16, "left_shoulder_yaw_joint": 17, "left_elbow_joint": 18, "left_wrist_roll_joint": 19, "left_wrist_pitch_joint": 20, "left_wrist_yaw_joint": 21,
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23, "right_shoulder_yaw_joint": 24, "right_elbow_joint": 25, "right_wrist_roll_joint": 26, "right_wrist_pitch_joint": 27, "right_wrist_yaw_joint": 28
}

class MuJoCoMapeoInteractivo(Node):
    def __init__(self):
        super().__init__('mujoco_mapeo_interactivo')
        
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)
        
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ia_addr = ('127.0.0.1', 9876)
        
        self.current_q = {i: 0.0 for i in range(29)}
        self.state_received = False
        self.dt = 0.02

        # Diccionario que mantendrá los motores que tú llames explícitamente
        self.targets = {}
        
        # ── EL LATIDO (HEARTBEAT) ──
        # Este hilo asegura que los motores se queden de piedra donde tú digas,
        # protegiéndolos de otras IAs o planificadores.
        threading.Thread(target=self.udp_publisher_loop, daemon=True).start()

    def udp_publisher_loop(self):
        """Publica el estado objetivo constantemente a 50Hz."""
        while True:
            if self.targets:
                payload = {str(k): float(v) for k, v in self.targets.items()}
                self.udp_sock.sendto(json.dumps(payload).encode(), self.ia_addr)
            time.sleep(0.02)

    def state_callback(self, msg):
        # Actualiza la pose actual en tiempo real a prueba de fallos
        for i, name in enumerate(msg.name):
            if name in FULL_JOINT_MAP:
                motor_id = FULL_JOINT_MAP[name]
                self.current_q[motor_id] = msg.position[i]
                
        self.state_received = True

    def move_to(self, motor_idx, target_q, duration=2.0):
        if not self.state_received:
            print("[ERROR] Esperando conexión con el simulador...")
            return

        steps = int(duration / self.dt)
        start_q = self.current_q[motor_idx]
        
        print(f"\n[INFO] Moviendo motor {motor_idx} de {start_q:.3f} a {target_q:.3f} rad...")

        # Bucle de interpolación (solo calcula, el envío lo hace el Heartbeat)
        for i in range(steps):
            alpha = i / steps
            current_val = start_q * (1 - alpha) + target_q * alpha
            self.targets[motor_idx] = current_val
            time.sleep(self.dt)
            
        # Clavamos el ángulo final exacto
        self.targets[motor_idx] = target_q
        print("[INFO] Movimiento completado. El robot mantendrá esta postura firme.")

    def release_control(self):
        """Vaciamos los targets y mandamos un JSON vacío para devolver el control."""
        self.targets = {}
        self.udp_sock.sendto(json.dumps({}).encode(), self.ia_addr)

def main(args=None):
    rclpy.init(args=args)
    nodo = MuJoCoMapeoInteractivo()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(nodo)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    print("Conectando con MuJoCo (esperando /joint_states)...")
    while not nodo.state_received:
        time.sleep(0.1)
    
    print("\n" + "="*60)
    print("🤖 MODO MAPEO PARA MUJOCO INICIADO 🤖")
    print("Cintura: 12 (Yaw), 13 (Roll), 14 (Pitch)")
    print("Brazo Izquierdo: 15 a 21 (18 es Codo)")
    print("Brazo Derecho:   22 a 28 (25 es Codo)")
    print("Escribe 'q' para salir y devolver el control a la IA.")
    print("="*60)

    try:
        while True:
            entrada_motor = input("\n> Número de motor (ej. 15): ")
            if entrada_motor.lower() == 'q':
                break
                
            entrada_angulo = input("> Ángulo en radianes (ej. 0.3 o -0.3): ")
            if entrada_angulo.lower() == 'q':
                break

            try:
                motor = int(entrada_motor)
                angulo = float(entrada_angulo)
                
                if motor not in range(12, 29):
                    print("[ADVERTENCIA] Ese índice no pertenece a brazos/cintura (12-28).")
                    continue
                if angulo > 1.57 or angulo < -1.57:
                    print("[ADVERTENCIA] Por seguridad, limítate a +- 1.57 rad (90 grados).")
                    continue
                    
                nodo.move_to(motor, angulo)
                
            except ValueError:
                print("[ERROR] Por favor, introduce números válidos.")

    except KeyboardInterrupt:
        pass
    
    finally:
        print("\n[INFO] Devolviendo el control a la IA...")
        nodo.release_control()
        time.sleep(0.5)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
