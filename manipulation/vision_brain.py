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

os.environ["CYCLONEDDS_URI"] = """<CycloneDDS>
    <Domain>
        <SharedMemory>
            <Enable>false</Enable>
        </SharedMemory>
    </Domain>
</CycloneDDS>"""

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

context = zmq.Context()
video_socket = context.socket(zmq.SUB)
video_socket.setsockopt(zmq.CONFLATE, 1) 
video_socket.connect("tcp://127.0.0.1:5555")
video_socket.setsockopt_string(zmq.SUBSCRIBE, '')

ANCHO_CAJA_REAL = 0.20 
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
        
        self.use_6d = False
        self.target_rot = None 
        self.lock_elbows_wrists = False
        
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
        
        self.state_received = False
        ChannelFactoryInitialize(1, "lo") 
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.state_callback, 10)

    def state_callback(self, msg: LowState_):
        for i in range(29):
            self.current_jpos[i] = msg.motor_state[i].q
        if not self.state_received:
            self.state_received = True
            self.sync_math_with_reality()
            threading.Thread(target=self.control_loop, daemon=True).start()

    def sync_math_with_reality(self):
        for q_idx, g1_idx in self.pin_to_g1_q.items():
            self.q_math[q_idx] = self.current_jpos[g1_idx]
        pin.forwardKinematics(self.model, self.data, self.q_math)
        pin.updateFramePlacements(self.model, self.data)
        self.hand_xyz_actual = self.data.oMf[self.hand_frame_id].translation.copy()

    def generate_trajectory(self, start, end):
        dist = np.linalg.norm(end - start)
        num_steps = int(dist / 0.02) 
        if num_steps < 1: return [end]
        return [start + (i / num_steps) * (end - start) for i in range(1, num_steps + 1)]

    def set_target(self, raw_target_xyz):
        if not self.active_ik:
            self.sync_math_with_reality()
            start_pos = self.hand_xyz_actual
        else:
            start_pos = self.current_target_xyz if self.current_target_xyz is not None else self.hand_xyz_actual
            
        self.final_target_xyz = raw_target_xyz
        self.trajectory_points = self.generate_trajectory(start_pos, raw_target_xyz)
        if self.trajectory_points:
            self.current_target_xyz = self.trajectory_points.pop(0)
            self.active_ik = True

    def get_distance_to_target(self):
        if self.final_target_xyz is None: return 999.0
        return np.linalg.norm(self.final_target_xyz - self.hand_xyz_actual)

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

            err_xyz = self.current_target_xyz - self.hand_xyz_actual
            err_norm = np.linalg.norm(err_xyz)
            
            if err_norm < 0.01 and self.trajectory_points:
                self.current_target_xyz = self.trajectory_points.pop(0)
                err_xyz = self.current_target_xyz - self.hand_xyz_actual
                err_norm = np.linalg.norm(err_xyz)
            
            dq = np.zeros(self.model.nv)
            
            if self.use_6d and self.target_rot is not None:
                J = pin.computeFrameJacobian(self.model, self.data, self.q_math, self.hand_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J_arm = J[:, self.arm_v_indices] 
                
                if self.lock_elbows_wrists:
                    J_arm[:, 3:] = 0.0
                
                R_curr = self.data.oMf[self.hand_frame_id].rotation
                R_err = self.target_rot @ R_curr.T
                theta = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0))
                
                w = np.zeros(3)
                if theta > 1e-5:
                    w = (theta / (2 * np.sin(theta))) * np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
                
                err_6d = np.concatenate([err_xyz, w])
                norm_6d = np.linalg.norm(err_6d)
                
                if norm_6d > 0.04: err_6d = (err_6d / norm_6d) * 0.04
                
                pseudo_inv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + (0.05**2) * np.eye(6))
                dq_arm = pseudo_inv @ err_6d * 3.0
                for i, v_idx in enumerate(self.arm_v_indices): dq[v_idx] = dq_arm[i]

            else:
                if err_norm > 0.03: err_xyz = (err_xyz / err_norm) * 0.03
                if np.linalg.norm(err_xyz) > 0.005:
                    J = pin.computeFrameJacobian(self.model, self.data, self.q_math, self.hand_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                    J_arm = J[:3, self.arm_v_indices] 
                    
                    if self.lock_elbows_wrists:
                        J_arm[:, 3:] = 0.0
                        
                    pseudo_inv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + (0.05**2) * np.eye(3))
                    dq_arm = pseudo_inv @ err_xyz * 3.0
                    for i, v_idx in enumerate(self.arm_v_indices): dq[v_idx] = dq_arm[i]

            self.q_math = pin.integrate(self.model, self.q_math, dq * self.dt)

            comandos_brazos = {}
            for q_idx, g1_idx in self.pin_to_g1_q.items():
                comandos_brazos[g1_idx] = float(self.q_math[q_idx])
            
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

            time.sleep(max(0.0, self.dt - (time.time() - t_start)))

if __name__ == "__main__":
    print("🧠 Cerebro Maestro (FSM Completo + Anticrash + Levante + Retroceso) iniciado.")
    
    robot_ik = IntegratedIK()
    estado_robot = "BUSCANDO"
    estado_siguiente = ""
    ultimo_comando_walk = 0
    tiempo_llegada = 0
    memoria_caja = {} 

    while True:
        t_loop_start = time.time()
        try:
            buffer = video_socket.recv(flags=zmq.NOBLOCK)
            npimg = np.frombuffer(buffer, dtype=np.uint8)
            frame = cv2.imdecode(npimg, 1)
            
            if frame is None or frame.size == 0:
                continue
        except zmq.Again:
            time.sleep(0.01)
            continue
        except Exception as e:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        box_detected = False
        box_z, box_x_lat, box_y_vert = 0, 0, 0
        y_base_pixel = 0

        if contours:
            c_max = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c_max) > 100: 
                box_detected = True
                x, y, w, h = cv2.boundingRect(c_max)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                box_z = (ANCHO_CAJA_REAL * FOCAL_LENGTH) / w
                box_x_lat = (((x + w/2) - frame.shape[1]/2) * box_z) / FOCAL_LENGTH
                box_y_vert = (((y + h/2) - frame.shape[0]/2) * box_z) / FOCAL_LENGTH
                
                h_pixeles = h
                y_base_pixel = y + h
                
                cv2.putText(frame, f"Dist: {box_z:.2f}m", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        now = time.time()
        
        if estado_robot == "FINALIZADO":
            pass 

        # --- MÁQUINA DE ESTADOS FINITOS ---
        
        # 1. ACERCAMIENTO
        elif not box_detected and estado_robot in ["BUSCANDO", "ACERCANDO"]:
            if estado_robot != "BUSCANDO":
                print("👁️ Perdí la caja de vista. Buscando...")
                estado_robot = "BUSCANDO"
            if now - ultimo_comando_walk > 0.1:
                send_walk_cmd('q')
                ultimo_comando_walk = now
                
        elif box_detected and estado_robot in ["BUSCANDO", "ACERCANDO"]:
            # Reducido el acercamiento a 30cm para evitar choques con la mesa
            if box_z > 0.35: 
                estado_robot = "ACERCANDO"
                if now - ultimo_comando_walk > 0.1:
                    if box_x_lat > 0.05: send_walk_cmd('e')
                    elif box_x_lat < -0.05: send_walk_cmd('q')
                    else: send_walk_cmd('w')
                    ultimo_comando_walk = now
            else:
                print(f"\n🛑 ¡Distancia alcanzada ({box_z:.2f}m)! FRENANDO.")
                send_walk_cmd('stop')
                estado_robot = "PAUSA"
                estado_siguiente = "CALCULAR_Y_PREPARAR"
                tiempo_llegada = now

        # 2. PAUSA UNIVERSAL (1 SEGUNDO)
        elif estado_robot == "PAUSA":
            if now - tiempo_llegada > 1.0:
                estado_robot = estado_siguiente
                tiempo_llegada = now

        # 3. RAZONAMIENTO Y PREPARACIÓN DE BRAZOS
        elif estado_robot == "CALCULAR_Y_PREPARAR":
            print("📸 Razonamiento espacial en proceso...")
            alto_caja_real = (h_pixeles * box_z) / FOCAL_LENGTH
            centro_caja_base = transform_camera_to_base(box_z, box_x_lat, box_y_vert)
            mesa_y_vert_cam = ((y_base_pixel - frame.shape[0]/2) * box_z) / FOCAL_LENGTH
            z_mesa = transform_camera_to_base(box_z, box_x_lat, mesa_y_vert_cam)[2]
            
            memoria_caja = {
                'centro': centro_caja_base,
                'z_mesa': z_mesa,
                'alto': alto_caja_real
            }
            
            robot_ik.use_6d = False
            robot_ik.lock_elbows_wrists = False
            
            target = np.array([memoria_caja['centro'][0]- 0.1, memoria_caja['centro'][1] + (ANCHO_CAJA_REAL) + 0.15, memoria_caja['z_mesa'] + 0.15])
            robot_ik.set_target(target)
            
            estado_robot = "MOVIENDO_PREPARAR"
            tiempo_llegada = now

        elif estado_robot == "MOVIENDO_PREPARAR":
            error_dist = robot_ik.get_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_llegada > 4.0):
                print("✅ Brazos preparados.")
                estado_robot = "PAUSA"
                estado_siguiente = "INICIAR_ALINEAR"
                tiempo_llegada = now

        # 4. ALINEACIÓN (Modo Pinza Paralela 6D)
        elif estado_robot == "INICIAR_ALINEAR":
            robot_ik.target_rot = robot_ik.data.oMf[robot_ik.hand_frame_id].rotation.copy()
            robot_ik.use_6d = True 
            robot_ik.lock_elbows_wrists = False
            
            target = np.array([memoria_caja['centro'][0] + (ANCHO_CAJA_REAL/2), memoria_caja['centro'][1] + (ANCHO_CAJA_REAL/2) + 0.08, memoria_caja['z_mesa'] + 0.15])
            robot_ik.set_target(target)
            
            estado_robot = "MOVIENDO_ALINEAR"
            tiempo_llegada = now

        elif estado_robot == "MOVIENDO_ALINEAR":
            error_dist = robot_ik.get_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_llegada > 4.0):
                print("✅ Brazos alineados. Iniciando cierre...")
                estado_robot = "PAUSA"
                estado_siguiente = "MOVIENDO_AGARRAR"
                tiempo_llegada = now

        # 5. AGARRE INTELIGENTE (Propiocepción + Contacto)
        elif estado_robot == "MOVIENDO_AGARRAR":
            mano_y_actual = robot_ik.hand_xyz_actual[1]
            centro_caja_y = memoria_caja['centro'][1]
            
            borde_deseado_y = centro_caja_y + (ANCHO_CAJA_REAL / 2)
            distancia_al_borde = mano_y_actual - borde_deseado_y
            
            if distancia_al_borde < 0.02 or (now - tiempo_llegada > 3.0): 
                print("✅ Agarre físico/virtual asegurado.")
                estado_robot = "PAUSA"
                estado_siguiente = "INICIAR_LEVANTE"
                tiempo_llegada = now
            else:
                if not robot_ik.trajectory_points:
                    paso_seguro = min(0.01, distancia_al_borde) 
                    target = np.array([
                        memoria_caja['centro'][0] + (ANCHO_CAJA_REAL/2), 
                        mano_y_actual - paso_seguro, 
                        memoria_caja['z_mesa'] + 0.15
                    ])
                    robot_ik.set_target(target)

        # 6. LEVANTAMIENTO EN Z
        elif estado_robot == "INICIAR_LEVANTE":
            print("⬆️ Levantando la caja 10cm...")
            
            # Cogemos la posición matemática exacta actual y subimos 10cm en Z puro
            target = robot_ik.hand_xyz_actual.copy()
            target[2] += 0.10
            robot_ik.set_target(target)
            
            estado_robot = "MOVIENDO_LEVANTE"
            tiempo_llegada = now

        elif estado_robot == "MOVIENDO_LEVANTE":
            error_dist = robot_ik.get_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_llegada > 4.0):
                print("✅ Caja levantada.")
                estado_robot = "PAUSA"
                estado_siguiente = "RETROCEDER"
                tiempo_llegada = now
                ultimo_comando_walk = 0

        # 7. RETROCESO Y FINALIZACIÓN
        elif estado_robot == "RETROCEDER":
            if now - tiempo_llegada < 4.0:
                if now - ultimo_comando_walk > 0.1:
                    send_walk_cmd('s')
                    ultimo_comando_walk = now
            else:
                print("🎉 ¡MISIÓN CUMPLIDA! Caja extraída y asegurada.")
                send_walk_cmd('stop')
                estado_robot = "FINALIZADO"

        cv2.imshow("Cerebro Maestro G1", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        time.sleep(max(0, 0.02 - (time.time() - t_loop_start)))

cv2.destroyAllWindows()
