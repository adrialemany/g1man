import sys  # <-- Importante para forzar la salida
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data # <-- QoS optimizado para alta frecuencia
from unitree_hg.msg import LowState
import pinocchio as pin

class G1XYZReader(Node):
    def __init__(self):
        super().__init__('g1_xyz_reader')
        
        # Usamos qos_profile_sensor_data: tamaño de cola pequeño y 'Best Effort'.
        # Así evitamos que la RAM o el procesador se saturen de mensajes antiguos.
        self.state_sub = self.create_subscription(
            LowState, 
            '/lowstate', 
            self.state_callback, 
            qos_profile_sensor_data
        )
        
        # --- CONFIGURACIÓN DE PINOCCHIO ---
        urdf_path = "/home/unitree/robot_ws/src/g1pilot/description_files/urdf/g1_29dof.urdf"
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            self.hand_frame_id = self.model.getFrameId("left_rubber_hand")
        except Exception as e:
            self.get_logger().error(f"Error cargando URDF: {e}")
            self.model = None

        # Mapeo de los motores del brazo izquierdo
        self.joint_mapping = {
            'left_shoulder_pitch_joint': 15,
            'left_shoulder_roll_joint': 16,
            'left_shoulder_yaw_joint': 17,
            'left_elbow_joint': 18,        
            'left_wrist_roll_joint': 19
        }
        
        self.timer = self.create_timer(0.5, self.print_xyz)
        self.low_state = None

    def state_callback(self, msg):
        self.low_state = msg

    def print_xyz(self):
        if self.model is None or self.low_state is None:
            return
            
        q = pin.neutral(self.model)
        
        for joint_name, motor_idx in self.joint_mapping.items():
            if self.model.existJointName(joint_name):
                pin_joint_id = self.model.getJointId(joint_name)
                q_idx = self.model.joints[pin_joint_id].idx_q
                q[q_idx] = self.low_state.motor_state[motor_idx].q

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        xyz = self.data.oMf[self.hand_frame_id].translation
        
        self.get_logger().info(f"📍 Mano Izquierda -> X: {xyz[0]:.3f} | Y: {xyz[1]:.3f} | Z: {xyz[2]:.3f} (metros)")

def main(args=None):
    rclpy.init(args=args)
    node = G1XYZReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Añadimos un pequeño print para que sepas que sí detectó el Ctrl+C
        node.get_logger().info("Ctrl+C detectado, cerrando nodo limpiamente...")
    finally:
        node.destroy_node()
        # Verificamos si ROS ya se apagó para no lanzar una excepción doble
        if rclpy.ok():
            rclpy.shutdown()
        # Forzamos la salida al sistema operativo para matar cualquier hilo rebelde
        sys.exit(0)

if __name__ == '__main__':
    main()
