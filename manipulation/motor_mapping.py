import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState
import time
import threading

class G1MapeoInteractivo(Node):
    def __init__(self):
        super().__init__('g1_mapeo_interactivo')
        
        self.NOT_USED_JOINT = 29 
        self.kp = 60.0
        self.kd = 1.5
        self.dt = 0.02
        
        self.cmd_pub = self.create_publisher(LowCmd, '/arm_sdk', 10)
        self.state_sub = self.create_subscription(LowState, '/lowstate', self.state_callback, 10)
        
        self.low_state = None
        self.state_received = False

    def state_callback(self, msg):
        # Mantenemos el estado actualizado en tiempo real
        self.low_state = msg
        self.state_received = True

    def move_to(self, motor_idx, target_q, duration=2.0):
        """Mueve un motor específico a un ángulo objetivo suavemente."""
        if not self.state_received:
            print("Esperando conexión con el robot...")
            return

        steps = int(duration / self.dt)
        start_q = self.low_state.motor_state[motor_idx].q
        
        print(f"\n[INFO] Moviendo motor {motor_idx} de {start_q:.3f} a {target_q:.3f} rad...")

        for i in range(steps):
            alpha = i / steps
            current_q = start_q * (1 - alpha) + target_q * alpha
            
            cmd = LowCmd()
            # Congelamos el resto del cuerpo en su posición EXACTA actual
            for j in range(29):
                cmd.motor_cmd[j].q = self.low_state.motor_state[j].q
                cmd.motor_cmd[j].kp = self.kp
                cmd.motor_cmd[j].kd = self.kd

            # Sobrescribimos solo el motor que estamos mapeando
            cmd.motor_cmd[motor_idx].q = current_q
            cmd.motor_cmd[self.NOT_USED_JOINT].q = 1.0
            
            self.cmd_pub.publish(cmd)
            time.sleep(self.dt)
            
        print("[INFO] Movimiento completado. Usa una regla/transportador para medir.")

def main(args=None):
    rclpy.init(args=args)
    nodo = G1MapeoInteractivo()
    
    # 1. Ejecutamos ROS 2 en un hilo en segundo plano (para poder usar el input del teclado)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(nodo)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # 2. Esperamos a que el nodo reciba datos
    print("Conectando con el G1...")
    while not nodo.state_received:
        time.sleep(0.1)
    
    print("\n" + "="*50)
    print("🤖 MODO MAPEO DE BRAZO INICIADO 🤖")
    print("Brazo Izquierdo: 15 (Hombro Pitch), 16 (Hombro Roll), 17 (Hombro Yaw), 18 (Codo), 19 (Muñeca)")
    print("Escribe 'q' para salir y soltar el brazo.")
    print("="*50)

    # 3. Bucle interactivo para el usuario
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
                
                # Límites de seguridad básicos para evitar que lo rompas testeando
                if motor not in range(12, 29):
                    print("[ADVERTENCIA] Ese índice no pertenece a brazos/cintura.")
                    continue
                if angulo > 1.57 or angulo < -1.57:
                    print("[ADVERTENCIA] Por seguridad, limítate a +- 1.57 rad (90 grados) para testear.")
                    continue
                    
                nodo.move_to(motor, angulo)
                
            except ValueError:
                print("[ERROR] Por favor, introduce números válidos.")

    except KeyboardInterrupt:
        pass
    
    finally:
        print("\nDevolviendo control al robot...")
        cmd = LowCmd()
        cmd.motor_cmd[nodo.NOT_USED_JOINT].q = 0.0
        nodo.cmd_pub.publish(cmd)
        time.sleep(0.5)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
