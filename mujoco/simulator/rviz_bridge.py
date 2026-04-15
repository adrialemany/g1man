import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import socket
import json
import time
import threading

JOINT_MAP = {
    # Brazo Derecho
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23,
    "right_shoulder_yaw_joint":   24, "right_elbow_joint":         25,
    "right_wrist_roll_joint":     26, "right_wrist_pitch_joint":   27,
    "right_wrist_yaw_joint":      28,
    # Brazo Izquierdo
    "left_shoulder_pitch_joint":  15, "left_shoulder_roll_joint":  16,
    "left_shoulder_yaw_joint":    17, "left_elbow_joint":          18,
    "left_wrist_roll_joint":      19, "left_wrist_pitch_joint":    20,
    "left_wrist_yaw_joint":       21,
}

class RVizMujocoBridge(Node):
    def __init__(self):
        super().__init__('rviz_mujoco_bridge')
        
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ia_addr = ('127.0.0.1', 9876)
        
        self.sub_r = self.create_subscription(JointTrajectoryControllerState, '/right_arm_controller/controller_state', self.state_cb, 10)
        self.sub_l = self.create_subscription(JointTrajectoryControllerState, '/left_arm_controller/controller_state', self.state_cb, 10)
        
        self.target_joints = {}
        
        # EL SEGURO: Empezamos dormidos para no machacar la postura de la IA
        self.active = False 
        
        self.get_logger().info("🔗 Lazo Cerrado RViz -> MuJoCo conectado.")
        self.get_logger().info("😴 Puente en espera... Los brazos los controla la IA hasta que des la orden en MoveIt.")

        threading.Thread(target=self.stream_to_mujoco, daemon=True).start()

    def state_cb(self, msg):
        if not msg.reference.positions: return
        
        # 1. Vigilar si MoveIt arranca un movimiento real
        if not self.active:
            if msg.reference.velocities:
                # Si la velocidad de cualquier motor supera el 1%, despertamos el puente
                if any(abs(v) > 0.01 for v in msg.reference.velocities):
                    self.active = True
                    self.get_logger().info("🚀 ¡Movimiento detectado! Tomando el control de los brazos.")
        
        # 2. Solo registrar la postura si el puente está activo
        if self.active:
            for i, name in enumerate(msg.joint_names):
                m_id = JOINT_MAP.get(name)
