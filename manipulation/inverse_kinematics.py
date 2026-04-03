import os
import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, LowState
import time
import threading
import numpy as np
import pinocchio as pin

class G1PerfectIK(Node):
    def __init__(self):
        super().__init__('g1_perfect_ik')
        
        self.NOT_USED_JOINT = 29 
        self.kp = 60.0
        self.kd = 1.5
        self.dt = 0.02  # 50Hz
        
        self.cmd_pub = self.create_publisher(LowCmd, '/arm_sdk', 10)
        self.state_sub = self.create_subscription(LowState, '/lowstate', self.state_callback, 10)
        
        self.low_state = None
        self.state_received = False
        
        self.g1_arm_left = [15, 16, 17, 18, 19, 20, 21]
        self.g1_waist = [12, 13, 14]
        self.current_jpos = [0.0] * 29 
        
        self.active_ik = False       
        self.target_xyz = None
        self.hand_xyz_actual = np.zeros(3) # Para mostrar por pantalla
        
        # --- 1. CARGAR MODELO ---
        urdf_path = os.path.expanduser("~/robot_ws/src/g1pilot/description_files/urdf/g1_29dof.urdf")
        try:
            self.full_model = pin.buildModelFromUrdf(urdf_path)
            
            joints_to_lock_names = [
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
                "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
                "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
            ]
            
            locked_joint_ids = [self.full_model.getJointId(j) for j in joints_to_lock_names if self.full_model.existJointName(j)]
            q_neutral = pin.neutral(self.full_model)
            
            self.model = pin.buildReducedModel(self.full_model, locked_joint_ids, q_neutral)
            self.data = self.model.createData()
            self.hand_frame_id = self.model.getFrameId("left_rubber_hand")
            
        except Exception as e:
            self.get_logger().error(f"Error cargando URDF: {e}")
            self.model = None

        self.arm_names = [
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ]
        
        # --- 2. MAPEO EXACTO DE POSICIONES (q) Y VELOCIDADES (v) ---
        self.pin_to_g1_q = {}
        self.arm_v_indices = [] # ¡El secreto para que no vuele!
        
        if self.model is not None:
            for i, name in enumerate(self.arm_names):
                if self.model.existJointName(name):
                    j_id = self.model.getJointId(name)
                    q_idx = self.model.joints[j_id].idx_q
                    v_idx = self.model.joints[j_id].idx_v
                    
                    self.pin_to_g1_q[q_idx] = self.g1_arm_left[i]
                    self.arm_v_indices.append(v_idx)

        self.q_math = pin.neutral(self.model) # Cuaternión correcto inicializado
        self.timer = self.create_timer(self.dt, self.control_loop)

    def state_callback(self, msg):
        self.low_state = msg
        for i in range(29):
            if i < len(self.low_state.motor_state):
                self.current_jpos[i] = self.low_state.motor_state[i].q
            
        if not self.state_received:
            self.state_received = True
            # Forzamos una actualización inmediata de hand_xyz_actual para imprimirlo
            self.sync_math_with_reality()
            threading.Thread(target=self.input_loop, daemon=True).start()
            
    def sync_math_with_reality(self):
        """Lee los motores físicos y actualiza Pinocchio para saber dónde está la mano"""
        for q_idx, g1_idx in self.pin_to_g1_q.items():
            self.q_math[q_idx] = self.current_jpos[g1_idx]
        pin.forwardKinematics(self.model, self.data, self.q_math)
        pin.updateFramePlacements(self.model, self.data)
        self.hand_xyz_actual = self.data.oMf[self.hand_frame_id].translation.copy()

    def input_loop(self):
        time.sleep(1) 
        while rclpy.ok():
            try:
                print("\n" + "="*50)
                if not self.active_ik:
                    print("🤖 ESTADO: ROBOT LIBRE.")
                    self.sync_math_with_reality() # Actualizamos coordenadas si lo has movido con la mano
                else:
                    print(f"🎯 ESTADO: EN RUTA. Objetivo -> {self.target_xyz}")
                
                # CHIVATO: Te decimos dónde está el brazo exactamente para que sepas qué coordenadas meter
                print(f"📍 Posición actual de la mano:  X: {self.hand_xyz_actual[0]:.3f}   Y: {self.hand_xyz_actual[1]:.3f}   Z: {self.hand_xyz_actual[2]:.3f}")
                
                x_str = input("\nIntroduce X (m) [o 'q' para salir, 's' para soltar brazo]: ")
                
                if x_str.lower() == 'q':
                    break
                if x_str.lower() == 's':
                    self.active_ik = False
                    print("BRAZO LIBERADO.")
                    continue
                    
                y_str = input("Introduce Y (m): ")
                z_str = input("Introduce Z (m): ")
                
                self.target_xyz = np.array([float(x_str), float(y_str), float(z_str)])
                
                if not self.active_ik:
                    self.sync_math_with_reality()
                    self.active_ik = True
                    
                print(f"🚀 ¡Objetivo fijado en {self.target_xyz}! Moviendo...")
                
            except ValueError:
                print("[ERROR] Solo números válidos.")
            except EOFError:
                break

    def control_loop(self):
        if not self.state_received or self.model is None:
            return

        cmd = LowCmd()

        if not self.active_ik:
            cmd.motor_cmd[self.NOT_USED_JOINT].q = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 1. ACTUALIZAR PINOCCHIO
        pin.forwardKinematics(self.model, self.data, self.q_math)
        pin.updateFramePlacements(self.model, self.data)
        self.hand_xyz_actual = self.data.oMf[self.hand_frame_id].translation

        # 2. CALCULAR ERROR (Distancia al objetivo)
        err = self.target_xyz - self.hand_xyz_actual
        err_norm = np.linalg.norm(err)
        
        # Maximizamos el salto a 3cm para que sea suave pero rápido
        max_step = 0.03
        if err_norm > max_step:
            err = (err / err_norm) * max_step

        # 3. JACOBIANO DEFENESTRADO (Solo del brazo)
        dq = np.zeros(self.model.nv) # Array de 13 ceros
        
        if err_norm > 0.005:
            J = pin.computeFrameJacobian(self.model, self.data, self.q_math, self.hand_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_pos = J[:3, :] # Matriz 3x13
            
            # MAGIA: Extraemos SOLO las 7 columnas del brazo. Ignoramos la base.
            J_arm = J_pos[:, self.arm_v_indices] # Matriz 3x7
            
            # Pseudo-inversa amortiguada
            lambda_dls = 0.05
            pseudo_inv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + (lambda_dls**2) * np.eye(3))
            
            # Obtenemos las velocidades de los 7 motores (Ganancia de 3.0)
            dq_arm = pseudo_inv @ err * 3.0
            
            # Colocamos las 7 velocidades calculadas en sus posiciones correctas de los 13 huecos
            for i, v_idx in enumerate(self.arm_v_indices):
                dq[v_idx] = dq_arm[i]

        # 4. INTEGRAR VELOCIDAD (Evita el error 14 vs 13)
        self.q_math = pin.integrate(self.model, self.q_math, dq * self.dt)

        # 5. MANDAR AL ROBOT
        for q_idx, g1_idx in self.pin_to_g1_q.items():
            cmd.motor_cmd[g1_idx].q = self.q_math[q_idx]
            cmd.motor_cmd[g1_idx].dq = 0.0
            cmd.motor_cmd[g1_idx].tau = 0.0
            cmd.motor_cmd[g1_idx].kp = self.kp
            cmd.motor_cmd[g1_idx].kd = self.kd

        for waist_idx in self.g1_waist:
            cmd.motor_cmd[waist_idx].q = self.current_jpos[waist_idx]
            cmd.motor_cmd[waist_idx].kp = self.kp * 4.0
            cmd.motor_cmd[waist_idx].kd = self.kd * 4.0

        cmd.motor_cmd[self.NOT_USED_JOINT].q = 1.0
        self.cmd_pub.publish(cmd)

    def release_control(self):
        print("\nDevolviendo el control al robot y saliendo...")
        cmd = LowCmd()
        cmd.motor_cmd[self.NOT_USED_JOINT].q = 0.0
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = G1PerfectIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.release_control()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
