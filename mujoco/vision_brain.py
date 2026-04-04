import os
import sys
import time
import math
import numpy as np
import pinocchio as pin
import json
import socket
import threading
import cv2
import zmq

# =========================================================================
# CONFIGURACIÓN DE RED (ZMQ para cámara, UDP/TCP para IA)
# =========================================================================
context = zmq.Context()
video_socket = context.socket(zmq.SUB)
video_socket.connect("tcp://127.0.0.1:5555")
video_socket.setsockopt_string(zmq.SUBSCRIBE, '')

# Constantes de visión
ANCHO_CAJA_REAL = 0.06 
FOCAL_LENGTH = 460.0 

def send_walk_cmd(cmd):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', 6000))
        s.sendall(cmd.encode('utf-8'))
        s.recv(1024)
        s.close()
    except: pass

def transform_camera_to_base(z_dist, x_lateral, y_vertical):
    offset_camara = np.array([0.06, 0.0, 0.15]) 
    x_base = offset_camara[0] + z_dist       
    y_base = offset_camara[1] - x_lateral    
    z_base = offset_camara[2] - y_vertical   
    return np.array([x_base, y_base, z_base])

# =========================================================================
# CLASE DE CINEMÁTICA INVERSA (Integrada)
# =========================================================================
class IntegratedIK:
    def __init__(self):
        self.dt = 0.02
        self.g1_arm_left = [15, 16, 17, 18, 19, 20, 21]
        self.g1_arm_right = [22, 23, 24, 25, 26, 27, 28] 
        self.current_jpos = [0.0] * 29 
        
        self.active_ik = False       
        self.trajectory_points = []
        self.current_target_xyz = None
        self.final_target_xyz = None
        
        self.hand_xyz_actual = np.zeros(3)
        self.safe_zone = self.load_safe_zone("left_arm_safe_zone.json")
        
        # --- PREVENCIÓN DE COLISIÓN CON LA MESA ---
        # Añadimos un suelo duro a la zona segura para que las manos no bajen de 0.52m (Altura mesa)
        if self.safe_zone['z_min'] < 0.52:
            self.safe_zone['z_min'] = 0.52
            print("🛡️ Zona segura ajustada automáticamente para evitar colisión con la mesa.")
        
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
            print(f"Error cargando URDF en IK: {e}")
            self.model = None

        self.arm_names = [
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ]
        
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
        
        # Necesitamos suscribirnos a LowState para que el IK sepa dónde están los brazos
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        ChannelFactoryInitialize(1, "lo") 
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.state_callback, 10)

    def load_safe_zone(self, file_path):
        if not os.path.exists(file_path):
            print(f"Error: Falta {file_path}. Creando una genérica por defecto.")
            return {"x_min": 0.1, "x_max": 0.6, "y_min": 0.1, "y_max": 0.5, "z_min": -0.2, "z_max": 0.5}
        with open(file_path, 'r') as f:
            return json.load(f)

    def state_callback(self, msg):
        for i in range(29):
            self.current_jpos[i] = msg.motor_state[i].q

    def update_kinematics(self):
        for q_idx, g1_idx in self.pin_to_g1_q.items():
            self.q_math[q_idx] = self.current_jpos[g1_idx]
        pin.forwardKinematics(self.model, self.data, self.q_math)
        pin.updateFramePlacements(self.model, self.data)
        self.hand_xyz_actual = self.data.oMf[self.hand_frame_id].translation.copy()

    def clamp_target_to_safe_zone(self, start_xyz, target_xyz):
        sz = self.safe_zone
        if (sz['x_min'] <= target_xyz[0] <= sz['x_max'] and
            sz['y_min'] <= target_xyz[1] <= sz['y_max'] and
            sz['z_min'] <= target_xyz[2] <= sz['z_max']):
            return target_xyz

        direction = target_xyz - start_xyz
        direction[direction == 0] = 1e-6
        t_x_min, t_x_max = (sz['x_min'] - start_xyz[0]) / direction[0], (sz['x_max'] - start_xyz[0]) / direction[0]
        t_y_min, t_y_max = (sz['y_min'] - start_xyz[1]) / direction[1], (sz['y_max'] - start_xyz[1]) / direction[1]
        t_z_min, t_z_max = (sz['z_min'] - start_xyz[2]) / direction[2], (sz['z_max'] - start_xyz[2]) / direction[2]
        t_exit = min(max(t_x_min, t_x_max), max(t_y_min, t_y_max), max(t_z_min, t_z_max))
        t_safe = max(0.0, max(0.0, min(1.0, t_exit)) - 0.001)
        return start_xyz + direction * t_safe

    def set_target(self, raw_target_xyz):
        self.update_kinematics()
        safe_target = self.clamp_target_to_safe_zone(self.hand_xyz_actual, raw_target_xyz)
        dist = np.linalg.norm(safe_target - self.hand_xyz_actual)
        num_steps = int(dist / 0.02) 
        if num_steps < 1: 
            self.trajectory_points = [safe_target]
        else:
            self.trajectory_points = [self.hand_xyz_actual + (i / num_steps) * (safe_target - self.hand_xyz_actual) for i in range(1, num_steps + 1)]
        
        self.final_target_xyz = safe_target
        if self.trajectory_points:
            self.current_target_xyz = self.trajectory_points.pop(0)
            self.active_ik = True

    def run_ik_step(self):
        if not self.active_ik or self.current_target_xyz is None or self.model is None:
            return

        self.update_kinematics()
        err = self.current_target_xyz - self.hand_xyz_actual
        err_norm = np.linalg.norm(err)
        
        if err_norm < 0.01 and self.trajectory_points:
            self.current_target_xyz = self.trajectory_points.pop(0)
            err = self.current_target_xyz - self.hand_xyz_actual
            err_norm = np.linalg.norm(err)
        
        if err_norm > 0.03: err = (err / err_norm) * 0.03
        dq = np.zeros(self.model.nv)
        
        if err_norm > 0.005:
            J = pin.computeFrameJacobian(self.model, self.data, self.q_math, self.hand_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_arm = J[:3, self.arm_v_indices] 
            pseudo_inv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + (0.05**2) * np.eye(3))
            dq_arm = pseudo_inv @ err * 3.0
            
            for i, v_idx in enumerate(self.arm_v_indices):
                dq[v_idx] = dq_arm[i]

        self.q_math = pin.integrate(self.model, self.q_math, dq * self.dt)
        
        comandos_brazos = {}
        for q_idx, g1_idx in self.pin_to_g1_q.items():
            comandos_brazos[g1_idx] = float(self.q_math[q_idx])
        
        # Modo Espejo
        for i in range(7):
            left_motor_id = self.g1_arm_left[i]
            right_motor_id = self.g1_arm_right[i]
            left_angle = comandos_brazos[left_motor_id]
            if i in [1, 2, 4, 6]:
                comandos_brazos[right_motor_id] = -left_angle
            else:
                comandos_brazos[right_motor_id] = left_angle
        
        try:
            self.udp_sock.sendto(json.dumps(comandos_brazos).encode('utf-8'), self.target_address)
        except Exception: pass

# =========================================================================
# BUCLE PRINCIPAL (Visión + IK)
# =========================================================================
if __name__ == "__main__":
    print("🧠 Cerebro Maestro (Visión + IK) iniciado.")
    
    robot_ik = IntegratedIK()
    estado_robot = "BUSCANDO"
    ultimo_comando_walk = 0
    tiempo_llegada = 0

    while True:
        t_loop_start = time.time()
        
        # --- 1. PROCESAR VISIÓN ---
        try:
            buffer = video_socket.recv(flags=zmq.NOBLOCK)
            npimg = np.frombuffer(buffer, dtype=np.uint8)
            frame = cv2.imdecode(npimg, 1)
        except zmq.Again:
            # Si no hay frame nuevo, ejecutamos el IK y esperamos
            robot_ik.run_ik_step()
            time.sleep(max(0, 0.02 - (time.time() - t_loop_start)))
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        box_detected = False
        box_z, box_x_lat, box_y_vert = 0, 0, 0

        if contours:
            c_max = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c_max) > 100: 
                box_detected = True
                x, y, w, h = cv2.boundingRect(c_max)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                box_z = (ANCHO_CAJA_REAL * FOCAL_LENGTH) / w
                box_x_lat = (((x + w/2) - frame.shape[1]/2) * box_z) / FOCAL_LENGTH
                box_y_vert = (((y + h/2) - frame.shape[0]/2) * box_z) / FOCAL_LENGTH
                cv2.putText(frame, f"Dist: {box_z:.2f}m", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # --- 2. LÓGICA DE COMPORTAMIENTO ---
        now = time.time()
        
        if not box_detected and estado_robot != "AGARRANDO":
            if estado_robot != "BUSCANDO":
                print("👁️ Perdí la caja de vista. Buscando...")
                estado_robot = "BUSCANDO"
            if now - ultimo_comando_walk > 0.1:
                send_walk_cmd('q')
                ultimo_comando_walk = now
                
        elif box_detected and estado_robot != "AGARRANDO":
            if box_z > 0.45: 
                estado_robot = "ACERCANDO"
                if now - ultimo_comando_walk > 0.1:
                    if box_x_lat > 0.05: send_walk_cmd('e')
                    elif box_x_lat < -0.05: send_walk_cmd('q')
                    else: send_walk_cmd('w')
                    ultimo_comando_walk = now
            else:
                print("\n🛑 ¡Distancia alcanzada! FRENANDO.")
                send_walk_cmd('stop')
                estado_robot = "AGARRANDO"
                tiempo_llegada = now
                
                # Transformamos coordenadas visuales a coordenadas IK
                obj_base = transform_camera_to_base(box_z, box_x_lat, box_y_vert)
                
                # Objetivo para la mano IZQUIERDA (El IK lo reflejará para la derecha)
                target = np.array([
                    obj_base[0],          # Adelante
                    obj_base[1] + 0.15,   # A la izquierda de la caja (margen para agarrar)
                    obj_base[2]           # Altura detectada visualmente
                ])
                print(f"🦾 Enviando brazos a: {target}")
                robot_ik.set_target(target)

        # --- 3. EJECUTAR CINEMÁTICA ---
        # Si estamos agarrando, el IK se actualiza continuamente en cada frame
        if estado_robot == "AGARRANDO":
            robot_ik.run_ik_step()
            
            # Opcional: Soltar después de 10 segundos
            if now - tiempo_llegada > 10.0:
                print("🔄 Volviendo a modo libre...")
                robot_ik.active_ik = False
                send_walk_cmd('s') # Mando al robot hacia atrás para separarse de la mesa
                estado_robot = "BUSCANDO"

        cv2.imshow("Cerebro Maestro G1", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Mantener frecuencia de IK a ~50Hz
        time.sleep(max(0, 0.02 - (time.time() - t_loop_start)))

cv2.destroyAllWindows()
