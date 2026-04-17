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

video_socket_depth = context.socket(zmq.SUB)
video_socket_depth.setsockopt(zmq.CONFLATE, 1)
video_socket_depth.connect("tcp://127.0.0.1:5556")
video_socket_depth.setsockopt_string(zmq.SUBSCRIBE, '')

ANCHO_CAJA_REAL       = 0.20
PROFUNDIDAD_CAJA_REAL = 0.20    # Profundidad real de la caja
LONGITUD_MANO         = 0.12    # Distancia de la muñeca (punto IK) al centro de la palma/dedos
FOCAL_LENGTH          = 460.0
CX, CY                = 320.0, 240.0
IMG_W, IMG_H          = 640, 480
OFFSET_CAMARA         = np.array([0.06, 0.0, 0.15])   # Posición cámara respecto a base robot

# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS GEOMÉTRICOS  (mundo ↔ píxel, seguridad Z desde depth)
# ─────────────────────────────────────────────────────────────────────────────

def transform_camera_to_base(z_dist, x_lateral, y_vertical):
    x_base = OFFSET_CAMARA[0] + z_dist
    y_base = OFFSET_CAMARA[1] - x_lateral
    z_base = OFFSET_CAMARA[2] - y_vertical
    return np.array([x_base, y_base, z_base])


def world_to_pixel(pos_base):
    """
    Proyecta una coordenada 3‑D en el frame base del robot de vuelta al
    píxel (u, v) de la imagen de la cámara.
    Devuelve (None, None) si la posición está detrás de la cámara.
    """
    z_cam  = pos_base[0] - OFFSET_CAMARA[0]   # profundidad cámara
    x_lat  = OFFSET_CAMARA[1] - pos_base[1]   # lateral cámara
    y_vert = OFFSET_CAMARA[2] - pos_base[2]   # vertical cámara
    if z_cam < 0.05:
        return None, None
    u = int(CX + (x_lat  * FOCAL_LENGTH) / z_cam)
    v = int(CY + (y_vert * FOCAL_LENGTH) / z_cam)
    u = int(np.clip(u, 0, IMG_W - 1))
    v = int(np.clip(v, 0, IMG_H - 1))
    return u, v


def get_depth_at_world_pos(pos_base, depth_frame, ventana=10):
    """
    Devuelve la profundidad de cámara (metros) en la zona proyectada de
    pos_base, usando la mediana de una ventana de píxeles.
    Devuelve None si no hay datos válidos.
    """
    if depth_frame is None:
        return None
    u, v = world_to_pixel(pos_base)
    if u is None:
        return None
    vals = []
    for du in range(-ventana, ventana + 1, 2):
        for dv in range(-ventana, ventana + 1, 2):
            pu = int(np.clip(u + du, 0, IMG_W - 1))
            pv = int(np.clip(v + dv, 0, IMG_H - 1))
            d  = depth_frame[pv, pu]
            if 0.05 < d < 4.0:
                vals.append(d)
    if not vals:
        return None
    return float(np.percentile(vals, 15))   # superficie más cercana


def z_suelo_bajo_mano(pos_mano_base, depth_frame, margen_mano=0.06):
    """
    Consulta el depth frame buscando la superficie debajo de la mano.
    Filtra los puntos que están a la misma altura o por encima de la mano
    para evitar que el robot detecte su propio brazo como un obstáculo.
    """
    if depth_frame is None:
        return None

    u, v = world_to_pixel(pos_mano_base)
    if u is None:
        return None

    vals = []
    # Ventana más estrecha para no detectar la caja por error
    for du in range(-10, 11, 4):
        for dv in range(10, 40, 4):  
            pu = int(np.clip(u + du, 0, IMG_W - 1))
            pv = int(np.clip(v + dv, 0, IMG_H - 1))
            d  = depth_frame[pv, pu]
            
            if 0.05 < d < 4.0:
                # Calculamos la Z real de ese pixel
                y_vert_cam = ((pv - CY) * d) / FOCAL_LENGTH
                z_base_punto = OFFSET_CAMARA[2] - y_vert_cam
                
                # FILTRO ANTI-AUTODETECCIÓN: Ignorar todo lo que esté por encima de la mano
                # o a menos de 8cm por debajo de ella (asumimos que es parte del brazo propio).
                if z_base_punto < pos_mano_base[2] - 0.08:
                    vals.append(z_base_punto)

    if len(vals) < 5:
        return None

    # Tomamos el percentil 85 de los valores Z válidos (la superficie más alta detectada debajo)
    z_base_superficie = float(np.percentile(vals, 85))

    return z_base_superficie + margen_mano


def medir_inclinacion_caja(bbox, depth_frame):
    """
    Mide la profundidad Z en la parte izquierda y derecha de la caja para
    saber si el robot la está mirando de frente o de lado/diagonal.
    Devuelve la diferencia Z_izq - Z_der.
    Si > 0, la izquierda está más lejos (la cara mira hacia la derecha).
    Devuelve None si no puede realizar la medición con seguridad.
    """
    if depth_frame is None or bbox is None:
        return None
    
    x, y, w, h = bbox
    y_c = int(y + h / 2)
    
    # Muestrear al 15% y 85% del ancho para capturar bien los extremos de la cara visible
    x_l = int(x + w * 0.15)
    x_r = int(x + w * 0.85)
    
    z_l_vals = []
    z_r_vals = []
    
    # Muestrear ventanas pequeñas alrededor de esos puntos
    for dy in range(-5, 6):
        for dx in range(-5, 6):
            py = int(np.clip(y_c + dy, 0, IMG_H - 1))
            px_l = int(np.clip(x_l + dx, 0, IMG_W - 1))
            px_r = int(np.clip(x_r + dx, 0, IMG_W - 1))
            
            if 0.05 < depth_frame[py, px_l] < 4.0:
                z_l_vals.append(depth_frame[py, px_l])
            if 0.05 < depth_frame[py, px_r] < 4.0:
                z_r_vals.append(depth_frame[py, px_r])
                
    if not z_l_vals or not z_r_vals:
        return None
        
    z_l = float(np.median(z_l_vals))
    z_r = float(np.median(z_r_vals))
    
    # Si algún valor es espurio (0.0), es que midió aire/sombra
    if z_l < 0.1 or z_r < 0.1:
        return None
        
    return z_l - z_r


def scan_table_robust(depth_frame, bbox, box_z_cam):
    """
    Escanea la zona de la mesa alrededor de la caja (debajo y ambos laterales)
    con mucho más muestreo que la versión original.
    Devuelve (z_cam_mesa, n_muestras).
    """
    if depth_frame is None:
        return box_z_cam, 0

    x, y, w, h = bbox
    samples = []

    # ── Zona debajo de la caja (lo más fiable) ──────────────────────────────
    for row_off in range(3, 90, 2):
        row = min(y + h + row_off, IMG_H - 1)
        for col_off in range(-w // 2, int(w * 1.5), 3):
            col = int(np.clip(x + col_off, 0, IMG_W - 1))
            d   = depth_frame[row, col]
            if 0.05 < d < box_z_cam * 1.5:
                samples.append(d)

    # ── Zona lateral izquierda de la caja (entrada brazo izq. robot) ────────
    for col_off in range(w // 2 + 5, w + 100, 3):
        col = int(np.clip(x - col_off, 0, IMG_W - 1))
        for row_off in range(-h // 3, h + 30, 3):
            row = int(np.clip(y + row_off, 0, IMG_H - 1))
            d   = depth_frame[row, col]
            if 0.05 < d < box_z_cam * 1.5:
                samples.append(d)

    # ── Zona lateral derecha de la caja (entrada brazo der. robot) ──────────
    for col_off in range(w // 2 + 5, w + 100, 3):
        col = int(np.clip(x + w + col_off, 0, IMG_W - 1))
        for row_off in range(-h // 3, h + 30, 3):
            row = int(np.clip(y + row_off, 0, IMG_H - 1))
            d   = depth_frame[row, col]
            if 0.05 < d < box_z_cam * 1.5:
                samples.append(d)

    if len(samples) < 20:
        return box_z_cam, len(samples)

    # Percentil bajo → superficie más cercana (la mesa)
    return float(np.percentile(samples, 15)), len(samples)


def clamp_z_con_depth(target_pos, depth_frame, limite_z_seguro, margen_extra=0.02):
    """
    Dado un target de posición, consulta el depth frame en esa zona
    y devuelve la Z corregida si el sensor detecta una superficie más
    alta de lo esperado.  Nunca baja de limite_z_seguro.
    """
    target = target_pos.copy()
    z_sensor = z_suelo_bajo_mano(target, depth_frame)
    z_floor  = limite_z_seguro

    if z_sensor is not None:
        # Si el sensor ve suelo más alto de lo calculado estáticamente, usamos ese
        z_floor = max(limite_z_seguro, z_sensor + margen_extra)

    target[2] = max(target[2], z_floor)
    return target, z_floor


# ─────────────────────────────────────────────────────────────────────────────

def send_walk_cmd(cmd):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)
        s.connect(('127.0.0.1', 6000))
        s.sendall(cmd.encode('utf-8'))
        s.recv(1024)
        s.close()
    except:
        pass


# ─────────────────────────────────────────────────────────────────────────────
#  IK INTEGRADA  (sin cambios en lógica interna, sólo la misma clase)
# ─────────────────────────────────────────────────────────────────────────────

class IntegratedIK:
    def __init__(self):
        self.dt = 0.02
        self.g1_arm_left  = [15, 16, 17, 18, 19, 20, 21]
        self.g1_arm_right = [22, 23, 24, 25, 26, 27, 28]
        self.current_jpos = [0.0] * 29

        self.active_ik     = False
        self.traj_l        = []
        self.traj_r        = []
        self.target_l      = None
        self.target_r      = None
        self.final_target_l = None
        self.final_target_r = None

        self.hand_l_actual = np.zeros(3)
        self.hand_r_actual = np.zeros(3)

        self.use_6d            = False
        self.target_rot_l      = None
        self.target_rot_r      = None
        self.lock_elbows_wrists = False
        self.lock_shoulder_roll = False

        urdf_path = os.path.expanduser(
            "~/robot_ws/src/g1pilot/description_files/urdf/g1_29dof.urdf")
        try:
            self.full_model = pin.buildModelFromUrdf(urdf_path)
            joints_to_lock_names = [
                "left_hip_pitch_joint",  "left_hip_roll_joint",   "left_hip_yaw_joint",
                "left_knee_joint",       "left_ankle_pitch_joint","left_ankle_roll_joint",
                "right_hip_pitch_joint", "right_hip_roll_joint",  "right_hip_yaw_joint",
                "right_knee_joint",      "right_ankle_pitch_joint","right_ankle_roll_joint",
                "waist_yaw_joint",       "waist_roll_joint",      "waist_pitch_joint",
            ]
            locked_joint_ids = [
                self.full_model.getJointId(j)
                for j in joints_to_lock_names
                if self.full_model.existJointName(j)
            ]
            q_neutral   = pin.neutral(self.full_model)
            self.model  = pin.buildReducedModel(self.full_model, locked_joint_ids, q_neutral)
            self.data   = self.model.createData()

            self.left_hand_id  = self.model.getFrameId("left_rubber_hand")
            self.right_hand_id = self.model.getFrameId("right_rubber_hand")

            self.q_idx_16 = self.model.joints[
                self.model.getJointId("left_shoulder_roll_joint")].idx_q
            self.q_idx_23 = self.model.joints[
                self.model.getJointId("right_shoulder_roll_joint")].idx_q

        except Exception as e:
            print(f"[IK] Error cargando modelo: {e}")
            self.model = None

        self.left_arm_names = [
            "left_shoulder_pitch_joint",  "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",    "left_elbow_joint",
            "left_wrist_roll_joint",      "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
        ]
        self.right_arm_names = [
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",   "right_elbow_joint",
            "right_wrist_roll_joint",     "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]

        self.pin_to_g1_q    = {}
        self.left_v_indices  = []
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

        self.q_math      = pin.neutral(self.model) if self.model else None
        self.udp_sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
        dist      = np.linalg.norm(end - start)
        num_steps = int(dist / 0.02)
        if num_steps < 1:
            return [end]
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
        if self.final_target_l is None or self.final_target_r is None:
            return 999.0
        err_l = np.linalg.norm(self.final_target_l - self.hand_l_actual)
        err_r = np.linalg.norm(self.final_target_r - self.hand_r_actual)
        return max(err_l, err_r)

    def _compute_arm_ik(self, target_xyz, hand_actual, frame_id,
                        v_indices, target_rot=None, traj_list=None,
                        current_target=None):
        err_xyz  = current_target - hand_actual
        err_norm = np.linalg.norm(err_xyz)

        if err_norm < 0.01 and traj_list:
            current_target = traj_list.pop(0)
            err_xyz  = current_target - hand_actual
            err_norm = np.linalg.norm(err_xyz)

        dq_arm = np.zeros(len(v_indices))

        if self.use_6d and target_rot is not None:
            J      = pin.computeFrameJacobian(
                self.model, self.data, self.q_math, frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_arm  = J[:, v_indices]

            if self.lock_elbows_wrists: J_arm[:, 3:] = 0.0
            if self.lock_shoulder_roll: J_arm[:, 1]  = 0.0

            R_curr = self.data.oMf[frame_id].rotation
            R_err  = target_rot @ R_curr.T
            theta  = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0))

            w = np.zeros(3)
            if theta > 1e-5:
                w = (theta / (2 * np.sin(theta))) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ])

            err_6d  = np.concatenate([err_xyz, w])
            norm_6d = np.linalg.norm(err_6d)
            if norm_6d > 0.04:
                err_6d = (err_6d / norm_6d) * 0.04

            pseudo_inv = J_arm.T @ np.linalg.inv(
                J_arm @ J_arm.T + (0.05 ** 2) * np.eye(6))
            dq_arm = pseudo_inv @ err_6d * 3.0

        else:
            if err_norm > 0.03:
                err_xyz = (err_xyz / err_norm) * 0.03
            if np.linalg.norm(err_xyz) > 0.005:
                J      = pin.computeFrameJacobian(
                    self.model, self.data, self.q_math, frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                J_arm  = J[:3, v_indices]

                if self.lock_elbows_wrists: J_arm[:, 3:] = 0.0
                if self.lock_shoulder_roll: J_arm[:, 1]  = 0.0

                pseudo_inv = J_arm.T @ np.linalg.inv(
                    J_arm @ J_arm.T + (0.05 ** 2) * np.eye(3))
                dq_arm = pseudo_inv @ err_xyz * 3.0

        return dq_arm, current_target

    def control_loop(self):
        while True:
            t_start = time.time()
            if not self.state_received or self.model is None:
                time.sleep(self.dt)
                continue

            if (self.active_ik
                    and self.target_l is not None
                    and self.target_r is not None):

                pin.forwardKinematics(self.model, self.data, self.q_math)
                pin.updateFramePlacements(self.model, self.data)
                self.hand_l_actual = self.data.oMf[self.left_hand_id].translation
                self.hand_r_actual = self.data.oMf[self.right_hand_id].translation

                dq = np.zeros(self.model.nv)

                dq_l, self.target_l = self._compute_arm_ik(
                    self.final_target_l, self.hand_l_actual,
                    self.left_hand_id, self.left_v_indices,
                    self.target_rot_l, self.traj_l, self.target_l)
                for i, v_idx in enumerate(self.left_v_indices):
                    dq[v_idx] = dq_l[i]

                dq_r, self.target_r = self._compute_arm_ik(
                    self.final_target_r, self.hand_r_actual,
                    self.right_hand_id, self.right_v_indices,
                    self.target_rot_r, self.traj_r, self.target_r)
                for i, v_idx in enumerate(self.right_v_indices):
                    dq[v_idx] = dq_r[i]

                self.q_math = pin.integrate(self.model, self.q_math, dq * self.dt)

            comandos_brazos = {}
            for q_idx, g1_idx in self.pin_to_g1_q.items():
                comandos_brazos[g1_idx] = float(self.q_math[q_idx])

            try:
                self.udp_sock.sendto(
                    json.dumps(comandos_brazos).encode('utf-8'),
                    self.target_address)
            except Exception:
                pass

            time.sleep(max(0.0, self.dt - (time.time() - t_start)))


# ─────────────────────────────────────────────────────────────────────────────
#  BUCLE PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Cerebro Maestro v2 iniciado — descenso vertical + guardián Z en tiempo real.")

    robot_ik = IntegratedIK()

    # ── Estado de la máquina ──────────────────────────────────────────────────
    estado_robot         = "BUSCANDO"
    ultimo_comando_walk  = 0
    tiempo_estado        = 0

    memoria_caja          = {}
    LIMITE_Z_SEGURO       = -999.0
    Z_SUELO_DINAMICO      = -999.0    # se actualiza frame a frame con el depth
    base_q16_apertura     = 0.0
    base_q23_apertura     = 0.0
    inclinacion_caja      = None      # Guardará la rotación de la cara frontal

    # Variables para el descenso vertical controlado
    z_descenso_actual     = 0.0       # Z actual durante BAJAR_MANOS_LENTO
    z_descenso_objetivo   = 0.0       # Z final que queremos alcanzar
    VELOCIDAD_DESCENSO    = 0.006     # m por tick (0.006 m × 50 Hz ≈ 0.3 m/s)
    descenso_bloqueado    = False     # True si el sensor frena el descenso
    intentos_descenso     = 0

    depth_frame_global    = None      # frame depth más reciente
    bbox_caja_global      = None      # (x, y, w, h) de la caja detectada

    nombre_ventana = "Cerebro Maestro G1 v2"
    cv2.namedWindow(nombre_ventana, cv2.WINDOW_AUTOSIZE)

    while True:
        t_loop_start = time.time()

        # ── Captura RGB ───────────────────────────────────────────────────────
        try:
            buffer_rgb = video_socket.recv(flags=zmq.NOBLOCK)
            frame = cv2.imdecode(np.frombuffer(buffer_rgb, dtype=np.uint8), 1)
        except zmq.Again:
            time.sleep(0.01)
            continue
        except:
            continue

        if frame is None:
            continue

        # ── Captura Depth ─────────────────────────────────────────────────────
        depth_frame = None
        try:
            buffer_depth = video_socket_depth.recv(flags=zmq.NOBLOCK)
            depth_frame  = np.frombuffer(
                buffer_depth, dtype=np.float32).reshape((IMG_H, IMG_W))
            depth_frame_global = depth_frame
        except:
            depth_frame = depth_frame_global   # usar último válido si no hay nuevo

        # ── Detección HSV de la caja ──────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([40, 50,  50]),
                           np.array([80, 255, 255]))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        box_detected = False
        box_z = box_x_lat = box_y_vert = 0.0
        x_c = y_c = h_pixeles = 0
        x = y = w = h = 0

        if contours:
            c_max = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c_max) > 100:
                box_detected   = True
                x, y, w, h     = cv2.boundingRect(c_max)
                bbox_caja_global = (x, y, w, h)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                x_c, y_c   = int(x + w / 2), int(y + h / 2)
                h_pixeles  = h

                if (depth_frame is not None
                        and not np.isnan(depth_frame[y_c, x_c])
                        and depth_frame[y_c, x_c] > 0.05):
                    box_z = float(depth_frame[y_c, x_c])
                    # Extraer inclinación si se puede
                    inclinacion_caja = medir_inclinacion_caja(bbox_caja_global, depth_frame)
                else:
                    box_z = (ANCHO_CAJA_REAL * FOCAL_LENGTH) / w
                    inclinacion_caja = None

                box_x_lat  = ((x_c - CX) * box_z) / FOCAL_LENGTH
                box_y_vert = ((y_c - CY) * box_z) / FOCAL_LENGTH

                cv2.putText(frame, f"X_Robot: {box_z:.2f}m",
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2)

        # ── Actualización dinámica del suelo (sólo cuando hay depth y caja) ──
        if (depth_frame is not None
                and bbox_caja_global is not None
                and LIMITE_Z_SEGURO != -999.0
                and robot_ik.state_received):

            # Proyectamos dónde están las manos ahora mismo y pedimos el depth
            z_l = z_suelo_bajo_mano(robot_ik.hand_l_actual, depth_frame, margen_mano=0.07)
            z_r = z_suelo_bajo_mano(robot_ik.hand_r_actual, depth_frame, margen_mano=0.07)

            candidatos = [v for v in [z_l, z_r] if v is not None]
            if candidatos:
                Z_SUELO_DINAMICO = max(candidatos)   # usamos el más alto = más conservador

        now = time.time()

        # ── Máquina de estados ────────────────────────────────────────────────

        if estado_robot == "BUSCANDO":
            if box_detected and box_z < 3.0:
                send_walk_cmd('stop')
                print("[ESTADO] Caja detectada. Pasando a CENTRANDO_ROTACION.")
                estado_robot = "CENTRANDO_ROTACION"
                tiempo_estado = now
            elif now - ultimo_comando_walk > 0.15:
                send_walk_cmd('q')
                ultimo_comando_walk = now

        elif estado_robot == "CENTRANDO_ROTACION":
            if not box_detected:
                print("[ESTADO] Caja perdida. Volviendo a BUSCANDO.")
                estado_robot = "BUSCANDO"
            elif abs(box_x_lat) > 0.04:
                if now - ultimo_comando_walk > 0.15:
                    send_walk_cmd('e' if box_x_lat > 0 else 'q')
                    ultimo_comando_walk = now
            else:
                send_walk_cmd('stop')
                print("[ESTADO] Rotación centrada. Pasando a ACERCAMIENTO_ORBITA.")
                estado_robot = "ACERCAMIENTO_ORBITA"

        # ─────────────────────────────────────────────────────────────────────
        # Fase 1 de acercamiento. Vamos recto hasta estar a 0.8m.
        # En ese punto pasaremos a la órbita inteligente de alineación.
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "ACERCAMIENTO_ORBITA":
            if not box_detected:
                print("[ESTADO] Caja perdida. Volviendo a BUSCANDO.")
                estado_robot = "BUSCANDO"
            elif abs(box_x_lat) > 0.15:
                # Si se desvía lateralmente mucho, vuelve a centrar
                print("[ESTADO] Desvío lateral. Volviendo a CENTRANDO_ROTACION.")
                estado_robot = "CENTRANDO_ROTACION"
            elif box_z <= 0.80:
                send_walk_cmd('stop')
                print("[ESTADO] Distancia de órbita alcanzada. Pasando a ALINEAR_CARA.")
                estado_robot = "ALINEAR_CARA"
                tiempo_estado = now
            else:
                if now - ultimo_comando_walk > 0.20:
                    send_walk_cmd('w')
                    ultimo_comando_walk = now

        # ─────────────────────────────────────────────────────────────────────
        # Órbita inteligente para garantizar que el robot mira la
        # caja ortogonalmente (de frente) comprobando las esquinas del frame.
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "ALINEAR_CARA":
            if not box_detected:
                print("[ESTADO] Caja perdida. Volviendo a BUSCANDO.")
                estado_robot = "BUSCANDO"
            elif abs(box_x_lat) > 0.15:
                # Si durante la órbita la caja se sale mucho del centro,
                # volvemos a apuntar hacia ella girando el cuerpo antes de seguir
                print("[ESTADO] Desvío lateral durante órbita. Volviendo a CENTRANDO_ROTACION.")
                estado_robot = "CENTRANDO_ROTACION"
            elif inclinacion_caja is None:
                # La lectura de profundidad en los bordes está fallando.
                # Nos acercamos un poco más para que se vea más clara la caja.
                if box_z > 0.45:
                    if now - ultimo_comando_walk > 0.20:
                        send_walk_cmd('w')
                        ultimo_comando_walk = now
                else:
                    # Demasiado cerca y no ve la inclinación, aborta órbita para no atascarse
                    print("[ESTADO] Inclinación no medible. Abortando órbita -> ACERCAMIENTO_FINAL.")
                    estado_robot = "ACERCAMIENTO_FINAL"
            else:
                # Tolerancia de 3.5 cm para dar por plana la cara
                if inclinacion_caja > 0.035:
                    # Lado izquierdo está más lejos (se ve la cara lateral derecha) -> Orbitar a la izquierda
                    if now - ultimo_comando_walk > 0.20:
                        send_walk_cmd('a')
                        ultimo_comando_walk = now
                elif inclinacion_caja < -0.035:
                    # Lado derecho está más lejos -> Orbitar a la derecha
                    if now - ultimo_comando_walk > 0.20:
                        send_walk_cmd('d')
                        ultimo_comando_walk = now
                else:
                    # Diferencia ínfima, estamos viendo la cara totalmente de frente
                    send_walk_cmd('stop')
                    print("[ESTADO] Cara alineada ortogonalmente. Pasando a ACERCAMIENTO_FINAL.")
                    estado_robot = "ACERCAMIENTO_FINAL"

        # ─────────────────────────────────────────────────────────────────────
        # Fase 2 de acercamiento. Vamos recto hasta la distancia final (0.35m).
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "ACERCAMIENTO_FINAL":
            if not box_detected:
                print("[ESTADO] Caja perdida. Volviendo a BUSCANDO.")
                estado_robot = "BUSCANDO"
            elif abs(box_x_lat) > 0.12:
                print("[ESTADO] Desvío lateral. Volviendo a CENTRANDO_ROTACION.")
                estado_robot = "CENTRANDO_ROTACION"
            elif box_z <= 0.35:
                send_walk_cmd('stop')
                print("[ESTADO] Distancia final alcanzada. Pasando a AJUSTE_FINAL.")
                estado_robot = "AJUSTE_FINAL"
                tiempo_estado = now
            else:
                if now - ultimo_comando_walk > 0.20:
                    send_walk_cmd('w')
                    ultimo_comando_walk = now

        # ─────────────────────────────────────────────────────────────────────
        # REFINADO: Ahora Ajuste Final comprueba primero Y (lateral), luego X (adelante),
        # y luego dZ (rotación) para quedar clavado ortogonalmente frente a la caja.
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "AJUSTE_FINAL":
            if not box_detected:
                print("[ESTADO] Caja perdida. Volviendo a BUSCANDO.")
                estado_robot = "BUSCANDO"
            elif abs(box_x_lat) > 0.10:
                if now - ultimo_comando_walk > 0.15:
                    send_walk_cmd('d' if box_x_lat > 0 else 'a')
                    ultimo_comando_walk = now
            elif box_z > 0.40:
                # Estamos demasiado lejos, hay que dar paso adelante
                if now - ultimo_comando_walk > 0.20:
                    send_walk_cmd('w')
                    ultimo_comando_walk = now
            # ELIMINADO EL PASO HACIA ATRÁS: El robot no retrocederá si está a menos de 0.25
            elif inclinacion_caja is not None and abs(inclinacion_caja) > 0.035:
                # Seguimos sin estar perfectamente alineados, orbitamos para corregir
                if inclinacion_caja > 0.035:
                    if now - ultimo_comando_walk > 0.20:
                        send_walk_cmd('a')
                        ultimo_comando_walk = now
                else:
                    if now - ultimo_comando_walk > 0.20:
                        send_walk_cmd('d')
                        ultimo_comando_walk = now
            else:
                send_walk_cmd('stop')
                print("[ESTADO] Ajuste final completado. Pasando a ESTABILIZANDO.")
                estado_robot = "ESTABILIZANDO"
                tiempo_estado = now

        elif estado_robot == "ESTABILIZANDO":
            if now - tiempo_estado > 2.5:
                # Doble validación: Si la inercia de la parada lo sacó de las marcas Y, X o Rotación
                if not box_detected:
                    print("[ESTADO] Caja perdida en estabilización. Volviendo a BUSCANDO.")
                    estado_robot = "BUSCANDO"
                elif abs(box_x_lat) > 0.10 or box_z > 0.40 or (inclinacion_caja is not None and abs(inclinacion_caja) > 0.04):
                    inclin_str = f"{inclinacion_caja:.3f}" if inclinacion_caja is not None else "---"
                    print(f"⚠️ Robot fuera de marca por inercia (Y_Robot:{box_x_lat:.2f}m, X_Robot:{box_z:.2f}m, dZ:{inclin_str}m). Reajustando...")
                    estado_robot = "AJUSTE_FINAL"
                else:
                    print("[ESTADO] Robot estabilizado perfectamente. Pasando a ANALIZANDO_ESCENA.")
                    estado_robot = "ANALIZANDO_ESCENA"

        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "ANALIZANDO_ESCENA":
            print("📸 Analizando escena con escaneo robusto de mesa...")

            alto_caja_real    = (h_pixeles * box_z) / FOCAL_LENGTH
            centro_caja_base  = transform_camera_to_base(box_z, box_x_lat, box_y_vert)

            # ── Escaneado robusto de la mesa ──────────────────────────────────
            z_mesa_cam, n_muestras = scan_table_robust(
                depth_frame, (x, y, w, h), box_z)
            print(f"   Muestras mesa: {n_muestras}  |  z_cam_mesa: {z_mesa_cam:.3f}m")

            # Convertir profundidad de cámara a Z base
            y_vert_mesa_cam    = ((y + h + 20 - CY) * z_mesa_cam) / FOCAL_LENGTH
            z_mesa_real        = transform_camera_to_base(
                z_mesa_cam, box_x_lat, y_vert_mesa_cam)[2]

            # ── Límite Z estático: con margen generoso ────────────────────────
            # Reducido a 5 cm sobre la mesa para permitir agarrar cajas más bajas
            LIMITE_Z_SEGURO = z_mesa_real + 0.05
            Z_SUELO_DINAMICO = LIMITE_Z_SEGURO   # inicializar dinámico igual al estático

            memoria_caja = {
                'centro': centro_caja_base,
                'z_mesa': z_mesa_real,
                'alto':   alto_caja_real,
                'bbox':   (x, y, w, h),
            }

            robot_ik.active_ik = False
            robot_ik.sync_math_with_reality()

            base_q16_apertura = robot_ik.q_math[robot_ik.q_idx_16]
            base_q23_apertura = robot_ik.q_math[robot_ik.q_idx_23]

            print(f"   z_mesa_real={z_mesa_real:.3f}  LIMITE_Z_SEGURO={LIMITE_Z_SEGURO:.3f}")
            print("[ESTADO] Escena analizada. Pasando a ABRIR_BRAZOS.")
            estado_robot  = "ABRIR_BRAZOS"
            tiempo_estado = now

        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "ABRIR_BRAZOS":
            progreso = min(1.0, (now - tiempo_estado) / 1.0)
            robot_ik.q_math[robot_ik.q_idx_16] = base_q16_apertura + (0.6 * progreso)
            robot_ik.q_math[robot_ik.q_idx_23] = base_q23_apertura - (0.6 * progreso)

            if progreso >= 1.0:
                print("👐 Brazos abiertos. Calculando IK Bimanual...")
                print("[ESTADO] Pasando a CALCULAR_Y_PREPARAR.")
                estado_robot  = "CALCULAR_Y_PREPARAR"
                tiempo_estado = now

        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "CALCULAR_Y_PREPARAR":
            robot_ik.use_6d            = False
            robot_ik.lock_elbows_wrists = False
            robot_ik.lock_shoulder_roll = True

            target_z = max(memoria_caja['z_mesa'] + 0.15, LIMITE_Z_SEGURO + 0.05)

            tgt_l = np.array([
                memoria_caja['centro'][0] - 0.1,  # Aquí no afecta porque es el punto de preparación
                memoria_caja['centro'][1] + (ANCHO_CAJA_REAL / 2) + 0.15,
                target_z,
            ])
            tgt_r = np.array([
                memoria_caja['centro'][0] - 0.1,
                memoria_caja['centro'][1] - (ANCHO_CAJA_REAL / 2) - 0.15,
                target_z,
            ])

            robot_ik.set_targets(tgt_l, tgt_r)
            print("[ESTADO] IK calculado. Pasando a MOVIENDO_PREPARAR.")
            estado_robot  = "MOVIENDO_PREPARAR"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_PREPARAR":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_estado > 4.0):
                print("[ESTADO] Posición de preparación alcanzada. Pasando a INICIAR_POSICION_SOBRE.")
                estado_robot  = "INICIAR_POSICION_SOBRE"
                tiempo_estado = now

        # ─────────────────────────────────────────────────────────────────────
        #  Posicionar manos DIRECTAMENTE SOBRE los puntos de agarre.
        #  Apuntando exactamente a la mitad de la caja,
        #  compensando la distancia física de la mano (LONGITUD_MANO).
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "INICIAR_POSICION_SOBRE":
            robot_ik.use_6d            = False
            robot_ik.lock_shoulder_roll = True

            # Z alta: bien por encima de la mesa
            z_sobre = max(LIMITE_Z_SEGURO + 0.18,
                          memoria_caja['centro'][2] + 0.12)

            # Eje X: Frente de la caja + (Mitad de profundidad) - (Longitud del gripper)
            tgt_x = memoria_caja['centro'][0] + (PROFUNDIDAD_CAJA_REAL / 2.0) - LONGITUD_MANO

            tgt_l = np.array([
                tgt_x,
                memoria_caja['centro'][1] + (ANCHO_CAJA_REAL / 2) + 0.15,  # <-- Mayor separación en Y
                z_sobre,
            ])
            tgt_r = np.array([
                tgt_x,
                memoria_caja['centro'][1] - (ANCHO_CAJA_REAL / 2) - 0.15,  # <-- Mayor separación en Y
                z_sobre,
            ])

            robot_ik.set_targets(tgt_l, tgt_r)
            print(f"🎯 Posicionando sobre la caja — z_sobre={z_sobre:.3f}")
            print("[ESTADO] Pasando a MOVIENDO_POSICION_SOBRE.")
            estado_robot  = "MOVIENDO_POSICION_SOBRE"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_POSICION_SOBRE":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_estado > 5.0):
                print("[ESTADO] Manos sobre la caja. Pasando a INICIAR_ALINEAR.")
                estado_robot  = "INICIAR_ALINEAR"
                tiempo_estado = now

        # ─────────────────────────────────────────────────────────────────────
        #  INICIAR_ALINEAR: Aplica rotación de palmas y se mantiene en Z alta y X media.
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "INICIAR_ALINEAR":
            rot_actual_l = robot_ik.data.oMf[robot_ik.left_hand_id].rotation.copy()
            rot_actual_r = robot_ik.data.oMf[robot_ik.right_hand_id].rotation.copy()

            # Rotación en el eje X (roll) para "levantar" las palmas y evitar 
            # que apunten hacia abajo por la flexión natural de los brazos.
            angulo_levantar = 0.8  # Radianes (~45 grados)
            R_ajuste_l = np.array([
                [1, 0, 0],
                [0, math.cos(-angulo_levantar), -math.sin(-angulo_levantar)],
                [0, math.sin(-angulo_levantar),  math.cos(-angulo_levantar)],
            ])
            R_ajuste_r = np.array([
                [1, 0, 0],
                [0, math.cos(angulo_levantar), -math.sin(angulo_levantar)],
                [0, math.sin(angulo_levantar),  math.cos(angulo_levantar)],
            ])

            robot_ik.target_rot_l  = R_ajuste_l @ rot_actual_l
            robot_ik.target_rot_r  = R_ajuste_r @ rot_actual_r
            robot_ik.use_6d        = True
            robot_ik.lock_shoulder_roll = True

            z_sobre = max(LIMITE_Z_SEGURO + 0.18,
                          memoria_caja['centro'][2] + 0.12)

            tgt_x = memoria_caja['centro'][0] + (PROFUNDIDAD_CAJA_REAL / 2.0) - LONGITUD_MANO

            tgt_l = np.array([
                tgt_x,
                memoria_caja['centro'][1] + (ANCHO_CAJA_REAL / 2) + 0.15,  # <-- Mayor separación en Y
                z_sobre,
            ])
            tgt_r = np.array([
                tgt_x,
                memoria_caja['centro'][1] - (ANCHO_CAJA_REAL / 2) - 0.15,  # <-- Mayor separación en Y
                z_sobre,
            ])

            robot_ik.set_targets(tgt_l, tgt_r)
            print("[ESTADO] Aplicando rotación de palmas. Pasando a MOVIENDO_ALINEAR.")
            estado_robot  = "MOVIENDO_ALINEAR"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_ALINEAR":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.05 or (now - tiempo_estado > 4.0):
                # Preparar variables para el descenso controlado
                z_descenso_actual   = robot_ik.hand_l_actual[2]
                
                # Bajar exactamente a la MITAD de la altura de la caja (entre su top y la mesa)
                z_mitad_caja = (memoria_caja['centro'][2] + memoria_caja['z_mesa']) / 2.0
                z_descenso_objetivo = max(z_mitad_caja, LIMITE_Z_SEGURO)
                
                descenso_bloqueado  = False
                intentos_descenso   = 0
                
                print(f"⬇️  Inicio descenso: {z_descenso_actual:.3f} → {z_descenso_objetivo:.3f} (Mitad de la caja)")
                print("[ESTADO] Palmas alineadas. Pasando a BAJAR_MANOS_LENTO.")
                estado_robot        = "BAJAR_MANOS_LENTO"
                tiempo_estado       = now

        # ─────────────────────────────────────────────────────────────────────
        #  BAJAR_MANOS_LENTO: Descenso vertical lento manteniendo X constante.
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "BAJAR_MANOS_LENTO":
            # Desactivamos el guardián dinámico aquí porque detecta los bordes de la caja
            # o los propios brazos y frena prematuramente en el borde superior.
            puede_bajar = (z_descenso_actual - VELOCIDAD_DESCENSO) >= z_descenso_objetivo

            if puede_bajar:
                z_descenso_actual -= VELOCIDAD_DESCENSO
                descenso_bloqueado = False
            else:
                z_descenso_actual = z_descenso_objetivo
                descenso_bloqueado = False  # Ya no se bloquea por falsos positivos

            # ── 3. Enviar target con la nueva Z y la X exacta de la mitad de caja
            if not robot_ik.traj_l and not robot_ik.traj_r:
                tgt_x = memoria_caja['centro'][0] + (PROFUNDIDAD_CAJA_REAL / 2.0) - LONGITUD_MANO
                tgt_l = np.array([
                    tgt_x,
                    memoria_caja['centro'][1] + (ANCHO_CAJA_REAL / 2) + 0.15,  # <-- Mayor separación en Y
                    z_descenso_actual,
                ])
                tgt_r = np.array([
                    tgt_x,
                    memoria_caja['centro'][1] - (ANCHO_CAJA_REAL / 2) - 0.15,  # <-- Mayor separación en Y
                    z_descenso_actual,
                ])
                robot_ik.set_targets(tgt_l, tgt_r)

            # ── 4. Condición de salida ────────────────────────────────────────
            ya_llegamos    = z_descenso_actual <= z_descenso_objetivo + 0.005
            timeout_bajada = now - tiempo_estado > 6.0

            if ya_llegamos or timeout_bajada:
                motivo = "LLEGADO" if ya_llegamos else "TIMEOUT"
                print(f"   Descenso completado [{motivo}] z_actual={z_descenso_actual:.3f} z_obj={z_descenso_objetivo:.3f}")
                print("[ESTADO] Descenso finalizado. Pasando a MOVIENDO_AGARRAR.")
                estado_robot  = "MOVIENDO_AGARRAR"
                tiempo_estado = now

        # ─────────────────────────────────────────────────────────────────────
        #  MOVIENDO_AGARRAR: Se acercan hacia adentro en la Y manteniendo la X constante.
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "MOVIENDO_AGARRAR":
            robot_ik.lock_shoulder_roll = False

            mano_l_y_actual = robot_ik.hand_l_actual[1]
            mano_r_y_actual = robot_ik.hand_r_actual[1]

            # El IK frame está en la muñeca. Para que la palma/dedos hagan contacto real,
            # el objetivo debe entrar un poco en la caja (compensando el grosor de la mano).
            PENETRACION_AGARRE = 0.045  # 4.5 cm hacia el centro de la caja desde el borde
            
            borde_l_y = memoria_caja['centro'][1] + (ANCHO_CAJA_REAL / 2) - PENETRACION_AGARRE
            borde_r_y = memoria_caja['centro'][1] - (ANCHO_CAJA_REAL / 2) + PENETRACION_AGARRE

            dist_l = mano_l_y_actual - borde_l_y
            dist_r = borde_r_y - mano_r_y_actual

            # Si ya hemos llegado al punto de agarre profundo o se agotó el tiempo
            if (dist_l < 0.01 and dist_r < 0.01) or (now - tiempo_estado > 16.0):
                print("[ESTADO] Agarre completado o timeout. Pasando a INICIAR_LEVANTE.")
                estado_robot  = "INICIAR_LEVANTE"
                tiempo_estado = now

            elif not robot_ik.traj_l and not robot_ik.traj_r:
                paso_seguro_l = min(0.01, max(0, dist_l))
                paso_seguro_r = min(0.01, max(0, dist_r))

                # Mantener la Z exactamente en el objetivo (mitad de la caja) 
                # en lugar de usar el guardián dinámico
                z_agarre = z_descenso_objetivo

                tgt_x = memoria_caja['centro'][0] + (PROFUNDIDAD_CAJA_REAL / 2.0) - LONGITUD_MANO
                tgt_l = np.array([
                    tgt_x,
                    mano_l_y_actual - paso_seguro_l,
                    z_agarre,
                ])
                tgt_r = np.array([
                    tgt_x,
                    mano_r_y_actual + paso_seguro_r,
                    z_agarre,
                ])

                robot_ik.set_targets(tgt_l, tgt_r)

        # ─────────────────────────────────────────────────────────────────────
        # Levantamiento en DOS FASES: 
        # FASE 1: Tirar de la caja hacia el estómago. Esto obliga a la cinemática
        # a doblar los codos de forma natural hacia atrás (evita la rotación invertida).
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "INICIAR_LEVANTE":
            print("[ESTADO] Fase 1 de Levante: Retrayendo brazos para forzar codos hacia atrás.")
            tgt_l = robot_ik.hand_l_actual.copy()
            tgt_r = robot_ik.hand_r_actual.copy()

            # X objetivo: Acercar la caja al pecho agresivamente (X = 0.20m)
            tgt_l[0] = 0.20
            tgt_r[0] = 0.20
            
            # Z objetivo: Subir solo un poquito para despegarla de la mesa (+5 cm)
            tgt_l[2] += 0.05
            tgt_r[2] += 0.05

            robot_ik.set_targets(tgt_l, tgt_r)
            print("[ESTADO] Pasando a MOVIENDO_LEVANTE_RETROCESO.")
            estado_robot  = "MOVIENDO_LEVANTE_RETROCESO"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_LEVANTE_RETROCESO":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_estado > 4.0):
                print("[ESTADO] Retracción completada. Codos asegurados. Pasando a Fase 2.")
                estado_robot       = "INICIAR_LEVANTE_VERTICAL"
                tiempo_estado      = now

        # ─────────────────────────────────────────────────────────────────────
        # FASE 2: Subir la caja en vertical. Al tener ya los codos plegados atrás, 
        # el IK simplemente rotará los hombros hacia arriba manteniendo la pose humana.
        # ─────────────────────────────────────────────────────────────────────
        elif estado_robot == "INICIAR_LEVANTE_VERTICAL":
            print("[ESTADO] Fase 2 de Levante: Subiendo la caja en altura (Z = 0.50m).")
            tgt_l = robot_ik.hand_l_actual.copy()
            tgt_r = robot_ik.hand_r_actual.copy()

            # X se mantiene en 0.20
            # Z objetivo: La altura final solicitada (0.50m)
            tgt_l[2] = 0.50
            tgt_r[2] = 0.50

            robot_ik.set_targets(tgt_l, tgt_r)
            print("[ESTADO] Pasando a MOVIENDO_LEVANTE_VERTICAL.")
            estado_robot  = "MOVIENDO_LEVANTE_VERTICAL"
            tiempo_estado = now

        elif estado_robot == "MOVIENDO_LEVANTE_VERTICAL":
            error_dist = robot_ik.get_max_distance_to_target()
            if error_dist < 0.04 or (now - tiempo_estado > 4.0):
                print("[ESTADO] Levantamiento vertical completado. Pasando a RETROCEDER.")
                estado_robot       = "RETROCEDER"
                tiempo_estado      = now
                ultimo_comando_walk = 0

        elif estado_robot == "RETROCEDER":
            if now - tiempo_estado < 2.0:
                if now - ultimo_comando_walk > 0.1:
                    send_walk_cmd('s')
                    ultimo_comando_walk = now
            else:
                send_walk_cmd('stop')
                if estado_robot != "FINALIZADO":
                    print("[ESTADO] Proceso finalizado con éxito.")
                    estado_robot = "FINALIZADO"

        # ── HUD ──────────────────────────────────────────────────────────────
        cv2.putText(frame, f"ESTADO: {estado_robot}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # Monitorizando la inclinación calculada
        if inclinacion_caja is not None:
            cv2.putText(frame, f"Inclinacion (dZ): {inclinacion_caja:.3f}m",
                        (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
        else:
            cv2.putText(frame, f"Inclinacion (dZ): ---",
                        (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)

        if LIMITE_Z_SEGURO != -999.0:
            z_din_str = f"{Z_SUELO_DINAMICO:.3f}" if Z_SUELO_DINAMICO != -999.0 else "---"
            cv2.putText(
                frame,
                f"Z_Mesa:{memoria_caja.get('z_mesa',0):.2f} "
                f"Z_Lim_Est:{LIMITE_Z_SEGURO:.2f} "
                f"Z_Din:{z_din_str}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # Dibujar proyección de manos sobre imagen (debug visual)
        if robot_ik.state_received and LIMITE_Z_SEGURO != -999.0:
            for mano, color in [(robot_ik.hand_l_actual, (255, 100, 0)),
                                 (robot_ik.hand_r_actual, (0, 100, 255))]:
                u_m, v_m = world_to_pixel(mano)
                if u_m is not None:
                    cv2.circle(frame, (u_m, v_m), 6, color, -1)

        cv2.imshow(nombre_ventana, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(max(0, 0.02 - (time.time() - t_loop_start)))

cv2.destroyAllWindows()
