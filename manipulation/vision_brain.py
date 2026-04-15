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

# Socket RGB
video_socket = context.socket(zmq.SUB)
video_socket.setsockopt(zmq.CONFLATE, 1) 
video_socket.connect("tcp://127.0.0.1:5555")
video_socket.setsockopt_string(zmq.SUBSCRIBE, '')

# Socket Depth (PointCloud)
video_socket_depth = context.socket(zmq.SUB)
video_socket_depth.setsockopt(zmq.CONFLATE, 1)
video_socket_depth.connect("tcp://127.0.0.1:5556")
video_socket_depth.setsockopt_string(zmq.SUBSCRIBE, '')

ANCHO_CAJA_REAL = 0.20 
FOCAL_LENGTH = 460.0 
CX, CY = 320.0, 240.0
IMG_W, IMG_H = 640, 480

def send_walk_cmd(cmd):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)
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
        
        # Trayectorias independientes
        self.traj_l = []
        self.traj_r = []
        self.target_l = None
        self.target_r = None
        self.final_target_l = None
        self.final_target_r = None
        
        self.hand_l_actual = np.zeros(3)
        self.hand_r_actual = np.zeros(3)
        
        self.use_6d = False
        self.target_rot_l = None 
        self.target_rot_r = None 
        self.lock_elbows_wrists = False
        self.lock_shoulder_roll = False 
        
        urdf_path = os.path.expanduser("~/robot_ws/src/g1pilot/description_files/urdf/g1_29dof.urdf")
        try:
            self.full_model = pin.buildModelFromUrdf(urdf_path)
            joints_to_lock_names = [
                "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
                "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
                "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"
            ]
            locked_joint_ids = [self.full_model.getJointId(j) for j in joints_to_lock_names if self.full_model.existJointName(j)]
            q_neutral = pin.neutral(self.full_model)
            self.model = pin.buildReducedModel(self.full_model, locked_joint_ids, q_neutral)
            self.data = self.model.createData()
            
            self.left_hand_id = self.model.getFrameId("left_rubber_hand")
            self.right_hand_id = self.model.getFrameId("right_rubber_hand")
            
            self.q_idx_16 = self.model.joints[self.model.getJointId("left_shoulder_roll_joint")].idx_q
            self.q_idx_23 = self.model.joints[self.model.getJointId("right_shoulder_roll_joint")].idx_q
            
        except Exception as e:
            self.model = None

        self.left_arm_names = [
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ]
        self.right_arm_names = [
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        ]
        
        self.pin_to_g1_q = {}
        self.left_v_indices = []
        self.right_v_indices = []
        
        if self.model is not None:
            for i, name in enumerate(self.left_arm_names):
                if self.model.existJointName(name):
                    j_id = self.model.getJointId(name)
                    self.pin_to_g1_q[self.model.joints[j_id].idx_q] = self.g1_arm_left[i]
                    self.left_v_indices.append(self.model.joints[j_id].idx_v)
            for i, name in enumerate(self.right_arm_names):
                if self.model.existJointName(name):
                    j_id = self.model.getJointId(name)
                    self.pin_to_g1_q[self.model.joints[j_id].idx_q] = self.g1_arm_right[i]
                    self.right_v_indices.append(self.model.joints[j_id].idx_v)

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
        self.hand_l_actual = self.data.oMf[self.left_hand_id].translation.copy()
        self.hand_r_actual = self.data.oMf[self.right_hand_id].translation.copy()

    def generate_trajectory(self, start, end):
        dist = np.linalg.norm(end - start)
        num_steps = int(dist / 0.02) 
        if num_steps < 1: return [end]
        return [start + (i / num_steps) * (end - start) for i in range(1, num_steps + 1)]

    def set_targets(self, raw_target_l, raw_target_r):
        if not self.active_ik:
            pin.forwardKinematics(self.model, self.data, self.q_math)
            pin.updateFramePlacements(self.model, self.data)
            self.hand_l_actual = self.data.oMf[self.left_hand_id].translation.copy()
            self.hand_r_actual = self.data.oMf[self.right_hand_id].translation.copy()
            start_l = self.hand_l_actual
            start_r = self.hand_r_actual
        else:
            start_l = self.target_l if self.target_l is not None else self.hand_l_actual
            start_r = self.target_r if self.target_r is not None else self.hand_r_actual
            
        self.final_target_l = raw_target_l
        self.final_target_r = raw_target_r
        
        self.traj_l = self.generate_trajectory(start_l, raw_target_l)
        self.traj_r = self.generate_trajectory(start_r, raw_target_r)
        
        if self.traj_l: self.target_l = self.traj_l.pop(0)
        if self.traj_r: self.target_r = self.traj_r.pop(0)
        self.active_ik = True

    def get_max_distance_to_target(self):
        if self.final_target_l is None or self.final_target_r is None: return 999.0
        err_l = np.linalg.norm(self.final_target_l - self.hand_l_actual)
        err_r = np.linalg.norm(self.final_target_r - self.hand_r_actual)
        return max(err_l, err_r)

    def _compute_arm_ik(self, target_xyz, hand_actual, frame_id, v_indices, target_rot=None, traj_list=None, current_target=None):
        err_xyz = current_target - hand_actual
        err_norm = np.linalg.norm(err_xyz)
        
        if err_norm < 0.01 and traj_list:
            current_target = traj_list.pop(0)
            err_xyz = current_target - hand_actual
            err_norm = np.linalg.norm(err_xyz)
            
        dq_arm = np.zeros(len(v_indices))
        
        if self.use_6d and target_rot is not None:
            J = pin.computeFrameJacobian(self.model, self.data, self.q_math, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_arm = J[:, v_indices] 
            
            if self.lock_elbows_wrists: J_arm[:, 3:] = 0.0
            if self.lock_shoulder_roll: J_arm[:, 1] = 0.0 
            
            R_curr = self.data.oMf[frame_id].rotation
            R_err = target_rot @ R_curr.T
            theta = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0))
            
            w = np.zeros(3)
            if theta > 1e-5:
                w = (theta / (2 * np.sin(theta))) * np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
            
            err_6d = np.concatenate([err_xyz, w])
            norm_6d = np.linalg.norm(err_6d)
            if norm_6d > 0.04: err_6d = (err_6d / norm_6d) * 0.04
            
            pseudo_inv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + (0.05**2) * np.eye(6))
            dq_arm = pseudo_inv @ err_6d * 3.0
            
        else:
            if err_norm > 0.03: err_xyz = (err_xyz / err_norm) * 0.03
            if np.linalg.norm(err_xyz) > 0.005:
                J = pin.computeFrameJacobian(self.model, self.data, self.q_math, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J_arm = J[:3, v_indices] 
                
                if self.lock_elbows_wrists: J_arm[:, 3:] = 0.0
                if self.lock_shoulder_roll: J_arm[:, 1] = 0.0 
                    
                pseudo_inv = J_arm.T @ np.linalg.inv(J_arm @ J_arm.T + (0.05**2) * np.eye(3))
                dq_arm = pseudo_inv @ err_xyz * 3.0

        return dq_arm, current_target

    def control_loop(self):
        while True:
            t_start = time.time()
            if not self.state_received or self.model is None: 
                time.sleep(self.dt)
                continue

            if self.active_ik and self.target_l is not None and self.target_r is not None:
                pin.forwardKinematics(self.model, self.data, self.q_math)
                pin.updateFramePlacements(self.model, self.data)
                self.hand_l_actual = self.data.oMf[self.left_hand_id].translation
                self.hand_r_actual = self.data.oMf[self.right_hand_id].translation

                dq = np.zeros(self.model.nv)
                
                # Calcular IK Brazo Izquierdo
                dq_l, self.target_l = self._compute_arm_ik(
                    self.final_target_l, self.hand_l_actual, self.left_hand_id, 
                    self.left_v_indices, self.target_rot_l, self.traj_l, self.target_l)
                for i, v_idx in enumerate(self.left_v_indices): dq[v_idx] = dq_l[i]
                
                # Calcular IK Brazo Derecho
                dq_r, self.target_r = self._compute_arm_ik(
                    self.final_target_r, self.hand_r_actual, self.right_hand_id, 
                    self.right_v_indices, self.target_rot_r, self.traj_r, self.target_r)
                for i, v_idx in enumerate(self.right_v_indices): dq[v_idx] = dq_r[i]

                self.q_math = pin.integrate(self.model, self.q_math, dq * self.dt)

            # Envío limpio por UDP a ambos brazos (Sin hack de espejo)
            comandos_brazos = {}
            for q_idx, g1_idx in self.pin_to_g1_q.items():
                comandos_brazos[g1_idx] = float(self.q_math[q_idx])
            
            try:
                self.udp_sock.sendto(json.dumps(comandos_brazos).encode('utf-8'), self.target_address)
            except Exception: pass

            time.sleep(max(0.0, self.dt - (time.time() - t_start)))

if __name__ == "__main__":
    print("Cerebro Maestro iniciado (IK Bimanual Paralela + Ajuste 0.1m de Mano).")
    
    robot_ik = IntegratedIK()
    estado_robot = "BUSCANDO"
    ultimo_comando_walk = 0
    tiempo_estado = 0
    
    memoria_caja = {} 
    LIMITE_Z_SEGURO = -999.0
    base_q16_apertura = 0.0 
    base_q23_apertura = 0.0 

    nombre_ventana = "Cerebro Maestro G1"
    cv2.namedWindow(nombre_ventana, cv2.WINDOW_AUTOSIZE)

    while True:
        t_loop_start = time.time()
        
        try:
            buffer_rgb = video_socket.recv(flags=zmq.NOBLOCK)
            frame = cv2.imdecode(np.frombuffer(buffer_rgb, dtype=np.uint8), 1)
        except zmq.Again:
            time.sleep(0.01)
            continue
        except: continue
        
        if frame is None: continue

        depth_frame = None
        try:
            buffer_depth = video_socket_depth.recv(flags=zmq.NOBLOCK)
            depth_frame = np.frombuffer(buffer_depth, dtype=np.float32).reshape((IMG_H, IMG_W))
        except: pass

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        box_detected = False
        box_z, box_x_lat, box_y_vert = 0.0, 0.0, 0.0
        x_c, y_c = 0, 0
        h_pixeles = 0

        if contours:
            c_max = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c_max) > 100: 
                box_detected = True
                x, y, w, h = cv2.boundingRect(c_max)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                x_c, y_c = int(x + w/2), int(y + h/2)
                h_pixeles = h
                
                if depth_frame is not None and not np.isnan(depth_frame[y_c, x_c]) and depth_frame[y_c, x_c] > 0.05:
                    box_z = float(depth_frame[y_c, x_c])
                else:
                    box_z = (ANCHO_CAJA_REAL * FOCAL_LENGTH) / w
                
                box_x_lat = ((x_c - CX) * box_z) / FOCAL_LENGTH
                box_y_vert = ((y_c - CY) * box_z) / FOCAL_LENGTH
                
                cv2.putText(frame, f"Z: {box_z:.2f}m", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        now = time.time()
        
        if estado_robot == "BUSCANDO":
            if box_detected and box_z < 3.0:
                send_walk_cmd('stop')
                estado_robot = "CENTRANDO_ROTACION"
                tiempo_estado = now
            elif now - ultimo_comando_walk > 0.15:
                send_walk_cmd('q')
                ultimo_comando_walk = now

        elif estado_robot == "CENTRANDO_ROTACION":
            if not box_detected:
                estado_robot = "BUSCANDO"
            elif abs(box_x_lat) > 0.04:
                if now - ultimo_comando_walk > 0.15:
                    send_walk_cmd('e' if box_x_lat > 0 else 'q')
                    ultimo_comando_walk = now
            else:
                send_walk_cmd('stop')
                estado_robot = "CAMINANDO_RECTO"

        elif estado_robot == "CAMINANDO_RECTO":
            if not box_detected:
                estado_robot = "BUSCANDO"
            elif box_z <= 0.35:
                send_walk_cmd('stop')
                estado_robot = "AJUSTE_FINAL"
                tiempo_estado = now
            else:
                if abs(box_x_lat) > 0.12 and now - ultimo_comando_walk > 0.20:
                    send_walk_cmd('d' if box_x_lat > 0 else 'a')
                    ultimo_comando_walk = now
                elif now - ultimo_comando_walk > 0.20:
                    send_walk_cmd('w')
                    ultimo_comando_walk = now

        elif estado_robot == "AJUSTE_FINAL":
            if abs(box_x_lat) > 0.02:
                if now - ultimo_comando_walk > 0.15:
                    send_walk_cmd('d' if box_x_lat > 0 else 'a')
                    ultimo_comando_walk = now
            else:
                send_walk_cmd('stop')
                estado_robot = "ESTABILIZANDO"
                tiempo_estado = now

        elif estado_robot == "ESTABILIZANDO":
            if now - tiempo_estado > 2.5:
                estado_robot = "ANALIZANDO_ESCENA"

        elif estado_robot == "ANALIZANDO_ESCENA":
            print("📸 Analizando Depth y calculando límites bimanuales...")
            alto_caja_real = (h_pixeles * box_z) / FOCAL_LENGTH
            centro_caja_base = transform_camera_to_base(box_z, box_x_lat, box_y_vert)
            
            z_mesa_cam = box_z 
            if depth_frame is not None and box_detected:
                v_tabla_min = min(y + h + 5, IMG_H - 1)
                v_tabla_max = min(y + h + 35, IMG_H - 1)
                franja = depth_frame[v_tabla_min:v_tabla_max, max(x-10, 0):min(x+w+10, IMG_W)]
                franja_valid = franja[(franja > 0.05) & (franja < 4.0)]
                if franja_valid.size > 5:
                    z_mesa_cam = float(np.median(franja_valid))
            
            mesa_y_vert_cam = ((y + h + 20 - CY) * z_mesa_cam) / FOCAL_LENGTH
            z_mesa_real = transform_camera_to_base(z_mesa_cam, box_x_lat, mesa_y_vert_cam)[2]
            
            # TAMAÑO MANO AJUSTADO A 0.1m
            LIMITE_Z_SEGURO = z_mesa_real + 0.08 + 0.02 

            memoria_caja = {
                'centro': centro_caja_base,
                'z_mesa': z_mesa_real,
                'alto': alto_caja_real
            }
            
            robot_ik.active_ik = False
            robot_ik.sync_math_with_reality()
            
            base_q16_apertura = robot_ik.q_math[robot_ik.q_idx_16]
            base_q23_apertura = robot_ik.q_math[robot_ik.q_idx_23]
            
            estado_robot = "ABRIR_BRAZOS"
            tiempo_estado = now

        elif estado_robot == "ABRIR_BRAZOS":
            progreso = min(1.0, (now - tiempo_estado) / 1.0)
            
            robot_ik.q_math[robot_ik.q_idx_16] = base_q16_apertura + (0.5 * progreso)
            robot_ik.q_math[robot_ik.q_idx_23] = base_q23_apertura - (0.5 * progreso)
            
            if progreso >= 1.0:
                print("👐 Brazos abiertos (+0.5 rad). Calculando IK Bimanual...")
                estado_robot = "CALCULAR_Y_PREPARAR"
                tiempo_estado = now

        elif estado_robot == "CALCULAR_Y_PREPARAR":
            robot_ik.use_6d = False
            robot_ik.lock_elbows_wrists = False
            robot_ik.lock_shoulder_roll = True 
            
            target_z = max(memoria_caja['z_mesa'] + 0.15, LIMITE_Z_SEGURO + 0.05)
            
            tgt_l = np.array([memoria_caja['centro'][0] - 0.1, memoria_caja['centro'][1] + (ANCHO_CAJA_REAL/2) + 0.15, target_z])
            tgt_r = np.array([memoria_caja['centro'][0] - 0.1, memoria_caja['centro'][1] - (ANCHO_CAJA_REAL/2) - 0.15, target_z])
            
            robot_ik.set_targets(tgt_l, tgt_r)
            
            estado_robot = "MOVIENDO_PREPARAR"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_PREPARAR":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_estado > 4.0):
                estado_robot = "INICIAR_ALINEAR"
                tiempo_estado = now

        elif estado_robot == "INICIAR_ALINEAR":
            rot_actual_l = robot_ik.data.oMf[robot_ik.left_hand_id].rotation.copy()
            rot_actual_r = robot_ik.data.oMf[robot_ik.right_hand_id].rotation.copy()
            
            # PALMAS PARALELAS: Rotación en Z para que apunten a los costados opuestos de la caja
            R_ajuste_l = np.array([[math.cos(-0.4), -math.sin(-0.4), 0], [math.sin(-0.4), math.cos(-0.4), 0], [0, 0, 1]])
            R_ajuste_r = np.array([[math.cos(0.4), -math.sin(0.4), 0], [math.sin(0.4), math.cos(0.4), 0], [0, 0, 1]])
            
            robot_ik.target_rot_l = R_ajuste_l @ rot_actual_l
            robot_ik.target_rot_r = R_ajuste_r @ rot_actual_r
            
            robot_ik.use_6d = True 
            robot_ik.lock_shoulder_roll = True 
            
            target_z = max(memoria_caja['centro'][2], LIMITE_Z_SEGURO)
            
            # Bajar paralelos a los lados
            tgt_l = np.array([memoria_caja['centro'][0] + 0.05, memoria_caja['centro'][1] + (ANCHO_CAJA_REAL/2) + 0.08, target_z])
            tgt_r = np.array([memoria_caja['centro'][0] + 0.05, memoria_caja['centro'][1] - (ANCHO_CAJA_REAL/2) - 0.08, target_z])
            
            robot_ik.set_targets(tgt_l, tgt_r)
            
            estado_robot = "MOVIENDO_ALINEAR"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_ALINEAR":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_estado > 4.0):
                estado_robot = "MOVIENDO_AGARRAR"
                tiempo_estado = now

        elif estado_robot == "MOVIENDO_AGARRAR":
            robot_ik.lock_shoulder_roll = False
            
            mano_l_y_actual = robot_ik.hand_l_actual[1]
            mano_r_y_actual = robot_ik.hand_r_actual[1]
            
            borde_l_y = memoria_caja['centro'][1] + (ANCHO_CAJA_REAL / 2)
            borde_r_y = memoria_caja['centro'][1] - (ANCHO_CAJA_REAL / 2)
            
            dist_l = mano_l_y_actual - borde_l_y
            dist_r = borde_r_y - mano_r_y_actual 
            
            if (dist_l < 0.01 and dist_r < 0.01) or (now - tiempo_estado > 6.0): 
                estado_robot = "INICIAR_LEVANTE"
                tiempo_estado = now
            else:
                if not robot_ik.traj_l and not robot_ik.traj_r:
                    # Mover hacia adentro manteniendo las palmas paralelas y respetando el Límite Z de la mesa
                    paso_seguro_l = min(0.01, max(0, dist_l))
                    paso_seguro_r = min(0.01, max(0, dist_r))
                    
                    target_z = max(memoria_caja['centro'][2], LIMITE_Z_SEGURO) 
                    
                    tgt_l = np.array([memoria_caja['centro'][0] + 0.05, mano_l_y_actual - paso_seguro_l, target_z])
                    tgt_r = np.array([memoria_caja['centro'][0] + 0.05, mano_r_y_actual + paso_seguro_r, target_z])
                    
                    robot_ik.set_targets(tgt_l, tgt_r)

        elif estado_robot == "INICIAR_LEVANTE":
            tgt_l = robot_ik.hand_l_actual.copy()
            tgt_r = robot_ik.hand_r_actual.copy()
            
            tgt_l[2] += 0.10; tgt_l[0] -= 0.15 
            tgt_r[2] += 0.10; tgt_r[0] -= 0.15 
            
            robot_ik.set_targets(tgt_l, tgt_r)
            
            estado_robot = "MOVIENDO_LEVANTE"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_LEVANTE":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_estado > 4.0):
                estado_robot = "RETROCEDER"
                tiempo_estado = now
                ultimo_comando_walk = 0

        elif estado_robot == "RETROCEDER":
            if now - tiempo_estado < 2.0:
                if now - ultimo_comando_walk > 0.1:
                    send_walk_cmd('s')
                    ultimo_comando_walk = now
            else:
                send_walk_cmd('stop')
                estado_robot = "FINALIZADO"

        cv2.putText(frame, f"ESTADO: {estado_robot}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        if LIMITE_Z_SEGURO != -999.0:
            cv2.putText(frame, f"Z_Mesa: {memoria_caja.get('z_mesa', 0):.2f}m | Z_Min_Mano: {LIMITE_Z_SEGURO:.2f}m", 
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.imshow(nombre_ventana, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        time.sleep(max(0, 0.02 - (time.time() - t_loop_start)))

cv2.destroyAllWindows()
