import sys
import os
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_hg.msg import LowState
import pinocchio as pin

class WorkspaceMapper(Node):
    def __init__(self):
        super().__init__('workspace_mapper')
        
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

        self.joint_mapping = {
            'left_shoulder_pitch_joint': 15,
            'left_shoulder_roll_joint': 16,
            'left_shoulder_yaw_joint': 17,
            'left_elbow_joint': 18,        
            'left_wrist_roll_joint': 19
        }
        
        # Variables para guardar los límites de la zona segura
        self.x_min = float('inf')
        self.x_max = float('-inf')
        self.y_min = float('inf')
        self.y_max = float('-inf')
        self.z_min = float('inf')
        self.z_max = float('-inf')
        
        self.timer = self.create_timer(0.1, self.record_limits) # A 10 Hz para capturar bien el movimiento
        self.low_state = None

    def state_callback(self, msg):
        self.low_state = msg

    def record_limits(self):
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
        
        x, y, z = xyz[0], xyz[1], xyz[2]
        
        # Actualizamos los límites si encontramos un valor más extremo
        self.x_min, self.x_max = min(self.x_min, x), max(self.x_max, x)
        self.y_min, self.y_max = min(self.y_min, y), max(self.y_max, y)
        self.z_min, self.z_max = min(self.z_min, z), max(self.z_max, z)
        
        # Imprimimos la caja actual (usamos \r para sobreescribir la misma línea en la terminal)
        sys.stdout.write(f"\r📦 Mapeando... X[{self.x_min:.3f}, {self.x_max:.3f}] | Y[{self.y_min:.3f}, {self.y_max:.3f}] | Z[{self.z_min:.3f}, {self.z_max:.3f}]")
        sys.stdout.flush()

    def save_and_exit(self):
        safe_zone = {
            'x_min': round(self.x_min, 3), 'x_max': round(self.x_max, 3),
            'y_min': round(self.y_min, 3), 'y_max': round(self.y_max, 3),
            'z_min': round(self.z_min, 3), 'z_max': round(self.z_max, 3)
        }
        
        # Guardamos en un archivo JSON en la carpeta actual, sobreescribiendo si existe
        file_path = "left_arm_safe_zone.json"
        with open(file_path, 'w') as f:
            json.dump(safe_zone, f, indent=4)
            
        print("\n\n" + "="*50)
        print("✅ ZONA SEGURA MAPEADA Y GUARDADA CON ÉXITO")
        print(f"Archivo sobreescrito/creado en: {os.path.abspath(file_path)}")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceMapper()
    
    print("🚀 Iniciando Mapeo de Zona Segura...")
    print("Mueve el brazo del robot por todos los bordes de la zona permitida.")
    print("Pulsa Ctrl+C cuando hayas terminado para sobreescribir el archivo JSON.\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_exit()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
