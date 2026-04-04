import time
import threading
import numpy as np
import pinocchio as pin
import json
import os
import sys
import socket

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

class G1PerfectIK:
    def __init__(self):
        self.NOT_USED_JOINT = 29 
        self.dt = 0.02
        
        self.g1_arm_left = [15, 16, 17, 18, 19, 20, 21]
        
        # --- AÑADIMOS EL BRAZO DERECHO PARA EL MODO ESPEJO ---
        self.g1_arm_right = [22, 23, 24, 25, 26, 27, 28] 
        
        self.g1_waist = [12, 13, 14]
        self.current_jpos = [0.0] * 29 
        
        self.active_ik = False       
        self.trajectory_points = []
        self.current_target_xyz = None
        self.final_target_xyz = None
        
        self.hand_xyz_actual = np.zeros(3)
        self.safe_zone = self.load_safe_zone("left_arm_safe_zone.json")
        
        self.joint_safety_limits = {
            15: (-3.04, 2.62), 16: (-1.54, 2.20), 17: (-2.57, 2.57),
            18: (-1.00, 2.04), 19: (-1.92, 1.92), 20: (-1.56, 1.56),
            21: (-1.56, 1.56)
        }
        
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
            print(f"Error cargando URDF: {e}")
            self.model = None

        self.arm_names = [
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ]

        if self.model is not None:
            print("\n--- LÍMITES FÍSICOS DEL URDF ---")
            for name in self.arm_names:
                if self.model.existJointName(name):
                    j_id = self.model.getJointId(name)
                    lim_inf = self.model.lowerPositionLimit[self.model.joints[j_id].idx_q]
                    lim_sup = self.model.upperPositionLimit[self.model.joints[j_id].idx_q]
                    print(f"Límites reales URDF para {name}: [{lim_inf:.2f}, {lim_sup:.2f}]")
            print("--------------------------------\n")
        
        self.pin_to_g1_q = {}
        self.arm_v_indices = []
        
        if self.model is not None:
            for i, name in enumerate(self.arm_names):
                if self.model.existJointName(name):
                    j_id = self.model.getJointId(name)
                    q_idx = self.model.joints[j_id].idx_q
                    v_idx = self.model.joints[j_id].idx_v
                    
                    self.pin_to_g1_q[q_idx] = self.g1_arm_left[i]
                    self.arm_v_indices.append(v_idx)

        self.q_math = pin.neutral(self.model)

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = ('127.0.0.1', 9876)

        self.state_received = False
        ChannelFactoryInitialize(1, "lo") 
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.state_callback, 10)
        
        print("[INFO] Esperando estado de la simulación...")

    def load_safe_zone(self, file_path):
        if not os.path.exists(file_path):
            print(f"Error: No hay {file_path}")
            sys.exit(1)
        with open(file_path, 'r') as f:
            zone = json.load(f)
        print("✅ Zona segura cargada.")
        return zone

    def state_callback(self, msg: LowState_):
        for i in range(29):
            self.current_jpos[i] = msg.motor_state[i].q
            
        if not self.state_received:
            self.state_received = True
            self.sync_math_with_reality()
            
            threading.Thread(target=self.control_loop, daemon=True).start()
            threading.Thread(target=self.input_loop, daemon=True).start()
            
    def sync_math_with_reality(self):
        for q_idx, g1_idx in self.pin_to_g1_q.items():
            self.q_math[q_idx] = self.current_jpos[g1_idx]
        pin.forwardKinematics(self.model, self.data, self.q_math)
        pin.updateFramePlacements(self.model, self.data)
        self.hand_xyz_actual = self.data.oMf[self.hand_frame_id].translation.copy()

    def check_joint_limits(self, q_state):
        for q_idx, g1_idx in self.pin_to_g1_q.items():
            if g1_idx in self.joint_safety_limits:
                min_q, max_q = self.joint_safety_limits[g1_idx]
                val = q_state[q_idx]
                if val < min_q or val > max_q:
                    print(f"\n🛑 LÍMITE: Motor {g1_idx} en {val:.2f}. Abortando.")
                    return False
        return True

    def clamp_target_to_safe_zone(self, start_xyz, target_xyz):
        sz = self.safe_zone
        if (sz['x_min'] <= target_xyz[0] <= sz['x_max'] and
            sz['y_min'] <= target_xyz[1] <= sz['y_max'] and
            sz['z_min'] <= target_xyz[2] <= sz['z_max']):
            return target_xyz

        print("\n⚠️ Objetivo fuera de zona segura. Intersecando...")
        direction = target_xyz - start_xyz
        direction[direction == 0] = 1e-6
        
        t_x_min, t_x_max = (sz['x_min'] - start_xyz[0]) / direction[0], (sz['x_max'] - start_xyz[0]) / direction[0]
        t_y_min, t_y_max = (sz['y_min'] - start_xyz[1]) / direction[1], (sz['y_max'] - start_xyz[1]) / direction[1]
        t_z_min, t_z_max = (sz['z_min'] - start_xyz[2]) / direction[2], (sz['z_max'] - start_xyz[2]) / direction[2]
        
        t_enter = max(min(t_x_min, t_x_max), min(t_y_min, t_y_max), min(t_z_min, t_z_max))
        t_exit = min(max(t_x_min, t_x_max), max(t_y_min, t_y_max), max(t_z_min, t_z_max))
        
        t_safe = max(0.0, max(0.0, min(1.0, t_exit)) - 0.001)
        return start_xyz + direction * t_safe

    def generate_trajectory(self, start, end):
        dist = np.linalg.norm(end - start)
        num_steps = int(dist / 0.02) 
        if num_steps < 1: return [end]
        return [start + (i / num_steps) * (end - start) for i in range(1, num_steps + 1)]

    def input_loop(self):
        time.sleep(0.5) 
        while True:
            try:
                print("\n" + "="*50)
                if not self.active_ik: print("🤖 ESTADO: ROBOT LIBRE.")
                else: print(f"🎯 ESTADO: EN RUTA hacia {self.final_target_xyz}")
                
                print(f"📍 Posición mano: X: {self.hand_xyz_actual[0]:.3f} Y: {self.hand_xyz_actual[1]:.3f} Z: {self.hand_xyz_actual[2]:.3f}")
                
                x_str = input("\nIntroduce X (m) [o 'q' salir, 's' soltar]: ")
                if x_str.lower() == 'q':
                    os._exit(0)
                if x_str.lower() == 's':
                    self.active_ik = False
                    self.trajectory_points = []
                    continue
                    
                y_str = input("Introduce Y (m): ")
                z_str = input("Introduce Z (m): ")
                
                raw_target_xyz = np.array([float(x_str), float(y_str), float(z_str)])
                self.sync_math_with_reality()
                safe_target = self.clamp_target_to_safe_zone(self.hand_xyz_actual, raw_target_xyz)
                self.final_target_xyz = safe_target
                self.trajectory_points = self.generate_trajectory(self.hand_xyz_actual, safe_target)
                
                if self.trajectory_points:
                    self.current_target_xyz = self.trajectory_points.pop(0)
                    self.active_ik = True
                    print(f"🚀 Generando trayectoria con Recálculo (Espacio Nulo) activado. Modo Espejo ON.")
            except ValueError:
                print("[ERROR] Solo números válidos.")
            except EOFError:
                break

    def control_loop(self):
        while True:
            t_start = time.time()
            if not self.state_received or self.model is None: 
                time.sleep(self.dt)
                continue

            if not self.active_ik or self.current_target_xyz is None:
                time.sleep(self.dt)
                continue

            pin.forwardKinematics(self.model, self.data, self.q_math)
            pin.updateFramePlacements(self.model, self.data)
            self.hand_xyz_actual = self.data.oMf[self.hand_frame_id].translation

            err = self.current_target_xyz - self.hand_xyz_actual
            err_norm = np.linalg.norm(err)
            
            if err_norm < 0.01: 
                if self.trajectory_points:
                    self.current_target_xyz = self.trajectory_points.pop(0)
                    err = self.current_target_xyz - self.hand_xyz_actual
                    err_norm = np.linalg.norm(err)
            
            max_step = 0.03
            if err_norm > max_step: err = (err / err_norm) * max_step

            dq = np.zeros(self.model.nv)
            
            if err_norm > 0.005:
                J = pin.computeFrameJacobian(self.model, self.data, self.q_math, self.hand_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J_pos = J[:3, :]
                J_arm = J_pos[:, self.arm_v_indices] 
                
                lambda_dls = 0.05
                pseudo_inv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + (lambda_dls**2) * np.eye(3))
                
                dq_primary = pseudo_inv @ err * 3.0
                
                dq_secondary = np.zeros(7)
                k_repulsion = 4.0 
                margin = 0.4 
                
                for i, name in enumerate(self.arm_names):
                    j_id = self.model.getJointId(name)
                    q_idx = self.model.joints[j_id].idx_q
                    g1_idx = self.g1_arm_left[i]

                    if g1_idx in self.joint_safety_limits:
                        min_q, max_q = self.joint_safety_limits[g1_idx]
                        current_q = self.q_math[q_idx]
                        
                        if current_q < (min_q + margin):
                            dq_secondary[i] = ((min_q + margin) - current_q) * k_repulsion
                        elif current_q > (max_q - margin):
                            dq_secondary[i] = ((max_q - margin) - current_q) * k_repulsion

                I = np.eye(7)
                null_space_projector = I - (pseudo_inv @ J_arm)
                dq_arm = dq_primary + (null_space_projector @ dq_secondary)

                for i, v_idx in enumerate(self.arm_v_indices):
                    dq[v_idx] = dq_arm[i]

            q_math_future = pin.integrate(self.model, self.q_math, dq * self.dt)

            if not self.check_joint_limits(q_math_future):
                self.active_ik = False
                self.trajectory_points = []
                self.current_target_xyz = None
                continue

            self.q_math = q_math_future

            comandos_brazos = {}
            # 1. Asignar brazo izquierdo
            for q_idx, g1_idx in self.pin_to_g1_q.items():
                comandos_brazos[g1_idx] = float(self.q_math[q_idx])
            
            # --- 2. LÓGICA DE MODO ESPEJO (BRAZO DERECHO) ---
            # Las articulaciones de los brazos del G1 tienen los siguientes índices en la lista:
            # [0]=Pitch, [1]=Roll, [2]=Yaw, [3]=Elbow, [4]=WristRoll, [5]=WristPitch, [6]=WristYaw
            
            for i in range(7):
                left_motor_id = self.g1_arm_left[i]
                right_motor_id = self.g1_arm_right[i]
                left_angle = comandos_brazos[left_motor_id]
                
                # Invertir el signo en Roll (Apertura lateral) y Yaw (Rotación sobre sí mismo)
                # Los índices 1 (Shoulder Roll), 2 (Shoulder Yaw), 4 (Wrist Roll) y 6 (Wrist Yaw) suelen invertirse
                if i in [1, 2, 4, 6]:
                    comandos_brazos[right_motor_id] = -left_angle
                else:
                    # Pitch (arriba/abajo) y Elbow (codo) mantienen el mismo signo
                    comandos_brazos[right_motor_id] = left_angle
            # ------------------------------------------------
            
            try:
                self.udp_sock.sendto(json.dumps(comandos_brazos).encode('utf-8'), self.target_address)
            except Exception:
                pass

            time.sleep(max(0.0, self.dt - (time.time() - t_start)))

if __name__ == '__main__':
    node = G1PerfectIK()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nSaliendo...")
        os._exit(0)
