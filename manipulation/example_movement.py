import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState
import math
import time

class G1ArmSdkController(Node):
    def __init__(self):
        super().__init__('g1_arm_sdk_controller')
        
        # En el G1 de 5 grados de libertad por brazo (G1ARM5), el índice del motor "no usado" 
        # que hace de interruptor de control es el 35. (En el G1ARM7 es el 43).
        self.NOT_USED_JOINT = 29 
        
        self.low_state = None
        self.state_received = False
        
        # PUBLICAMOS AL TOPIC DE LOS BRAZOS (/arm_sdk), NO AL GENERAL (/lowcmd)
        self.cmd_pub = self.create_publisher(LowCmd, '/arm_sdk', 10)
        
        # Nos suscribimos al estado general para leer dónde están los brazos ahora mismo
        self.state_sub = self.create_subscription(LowState, '/lowstate', self.state_callback, 10)
        
        self.dt = 0.02  # Bucle de control a 50Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.t = 0.0
        self.kp = 60.0
        self.kd = 1.5
        
        # Índices de los motores de los brazos (G1ARM5)
        self.arm_joints = [
            15, 16, 17, 18, 19,  # Brazo Izquierdo (Hombro Pitch, Roll, Yaw, Codo Pitch, Roll)
            22, 23, 24, 25, 26,  # Brazo Derecho
            12, 13, 14           # Cintura (Yaw, Roll, Pitch)
        ]
        
        self.current_jpos = [0.0] * len(self.arm_joints)
        self.get_logger().info("Esperando recibir el estado del robot...")

    def state_callback(self, msg):
        self.low_state = msg
        if not self.state_received:
            # Capturamos la posición inicial de los brazos solo la primera vez
            for i, joint_idx in enumerate(self.arm_joints):
                self.current_jpos[i] = self.low_state.motor_state[joint_idx].q
            self.state_received = True
            self.get_logger().info("Estado recibido. Tomando el control de los brazos...")

    def control_loop(self):
        if not self.state_received:
            return

        cmd = LowCmd()
        
        # Calculamos el movimiento (onda sinusoidal para el hombro izquierdo, índice 0 en nuestra lista)
        # Se moverá unos +- 1 radian desde su posición inicial
        target_shoulder_pitch = self.current_jpos[0] + 1.0 * math.sin(self.t)
        
        # Preparamos el comando para las articulaciones de los brazos
        for i, joint_idx in enumerate(self.arm_joints):
            # El hombro izquierdo (índice 0) lo movemos con la onda. 
            # El resto los mantenemos congelados en su posición inicial.
            if i == 0:
                cmd.motor_cmd[joint_idx].q = target_shoulder_pitch
            else:
                cmd.motor_cmd[joint_idx].q = self.current_jpos[i]
                
            cmd.motor_cmd[joint_idx].dq = 0.0
            cmd.motor_cmd[joint_idx].tau = 0.0
            cmd.motor_cmd[joint_idx].kp = self.kp
            cmd.motor_cmd[joint_idx].kd = self.kd
            
            # Nota: Según el C++ de Unitree, a la cintura le ponen más rigidez (x4)
            if joint_idx in [12, 13, 14]:
                cmd.motor_cmd[joint_idx].kp *= 4.0
                cmd.motor_cmd[joint_idx].kd *= 4.0

        # EL TRUCO DE MAGIA: Pedir el control de los brazos al cerebro (peso = 1.0)
        cmd.motor_cmd[self.NOT_USED_JOINT].q = 1.0
        
        self.cmd_pub.publish(cmd)
        self.t += self.dt

    def release_control(self):
        self.get_logger().info("Devolviendo el control de los brazos al robot...")
        cmd = LowCmd()
        # Ponemos el peso a 0 para que el robot vuelva a gobernar los brazos
        cmd.motor_cmd[self.NOT_USED_JOINT].q = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.1)  # Damos tiempo a que se publique antes de cerrar el nodo

def main(args=None):
    rclpy.init(args=args)
    node = G1ArmSdkController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.release_control()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
