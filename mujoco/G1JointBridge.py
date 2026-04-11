import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json

class G1JointBridge(Node):
    def __init__(self):
        super().__init__('g1_joint_bridge')

        # Escucha al robot fantasma de MoveIt/RViz
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Socket UDP para enviarle los comandos al Cerebro de la IA (Holosoma)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = ('127.0.0.1', 9876)

        # Mapeo de MoveIt a los IDs de los motores del G1
        self.joint_map = {
            "right_shoulder_pitch_joint": 22,
            "right_shoulder_roll_joint": 23,
            "right_shoulder_yaw_joint": 24,
            "right_elbow_joint": 25,
            "right_wrist_roll_joint": 26,
            "right_wrist_pitch_joint": 27,
            "right_wrist_yaw_joint": 28,
        }
        
        self.get_logger().info("🌉 Puente MoveIt -> UDP de la IA activado. (Las piernas están a salvo)")

    def joint_callback(self, msg: JointState):
        comandos_brazos = {}
        
        # Extraemos solo las articulaciones que nos interesan de MoveIt
        for i, name in enumerate(msg.name):
            if name in self.joint_map:
                motor_id = self.joint_map[name]
                # Convertimos la ID a string para que el JSON lo parsee bien al otro lado
                comandos_brazos[str(motor_id)] = float(msg.position[i])
        
        # Enviamos el diccionario por UDP a tu script de la IA
        if comandos_brazos:
            try:
                self.udp_sock.sendto(json.dumps(comandos_brazos).encode('utf-8'), self.target_address)
            except Exception as e:
                self.get_logger().error(f"Error enviando datos por UDP: {e}")

def main():
    rclpy.init()
    node = G1JointBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
