"""
moveit_brain.py  –  Cerebro completo para el Unitree G1 en MuJoCo.

Máquina de estados:
  REPOSO_BRAZOS        → Mantiene la pose por UDP.
  BUSCANDO             → Rota buscando la caja verde.
  CENTRANDO_ROTACION   → Gira sobre el sitio hasta centrar la caja.
  CAMINANDO_RECTO      → Avanza (distancia 0.20m) y corrige deriva lateral.
  AJUSTE_FINAL         → Strafe milimétrico para alinear a < 2 cm de error.
  ESTABILIZANDO        → Espera a que las físicas se asienten.
  ANALIZANDO_ESCENA    → Pre-chequea centrado. Fusiona RGB + Depth para calcular caja y mesa.
  EJECUTANDO_MANIOBRA  → BIMANUAL: ¡Ejecución nativa de MoveIt en lazo cerrado!
  FINALIZADO           → Misión completada.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import socket
import threading
import time
import numpy as np
import cv2
import zmq
import json
import os
import xml.etree.ElementTree as ET

from std_srvs.srv import Empty
from geometry_msgs.msg import Pose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    CollisionObject,
    PlanningScene,
)
from shape_msgs.msg import SolidPrimitive
# NUEVO: Importamos el estado en tiempo real de los controladores de MoveIt
from control_msgs.msg import JointTrajectoryControllerState 

# ───────────────────────────────────────────────────────────── CÁMARA
FOCAL_LENGTH = 460.0
CX, CY       = 320.0, 240.0
IMG_W, IMG_H = 640, 480

CAM_OFFSET_X =  0.06
CAM_OFFSET_Y =  0.00
CAM_OFFSET_Z =  0.15

# ─────────────────────────────────────── MAPEO JOINT NAME → ID MOTOR UDP
JOINT_MAP_R = {
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23,
    "right_shoulder_yaw_joint":   24, "right_elbow_joint":         25,
    "right_wrist_roll_joint":     26, "right_wrist_pitch_joint":   27,
    "right_wrist_yaw_joint":      28,
}
JOINT_MAP_L = {
    "left_shoulder_pitch_joint":  15, "left_shoulder_roll_joint":  16,
    "left_shoulder_yaw_joint":    17, "left_elbow_joint":          18,
    "left_wrist_roll_joint":      19, "left_wrist_pitch_joint":    20,
    "left_wrist_yaw_joint":       21,
}

def cam_to_pelvis(z_depth: float, x_lat: float, y_vert: float) -> np.ndarray:
    x = CAM_OFFSET_X + z_depth
    y = CAM_OFFSET_Y - x_lat
    z = CAM_OFFSET_Z - y_vert
    return np.array([x, y, z], dtype=np.float64)

class MoveItBrain(Node):

    def __init__(self):
        super().__init__('moveit_brain')

        # ── Clientes ROS 2
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._clear_octomap = self.create_client(Empty, '/clear_octomap')
        self._scene_pub     = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # ── EL PUENTE MÁGICO (Lazo Cerrado MoveIt -> MuJoCo) ──
        # Nos suscribimos al estado de los controladores virtuales de MoveIt
        self.sub_r = self.create_subscription(JointTrajectoryControllerState, '/right_arm_controller/controller_state', self._state_cb_r, 10)
        self.sub_l = self.create_subscription(JointTrajectoryControllerState, '/left_arm_controller/controller_state',  self._state_cb_l, 10)
        self.cmd_r = {}
        self.cmd_l = {}

        # ── Sockets UDP
        self.udp_arm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ia_addr = ('127.0.0.1', 9876)

        # ── ZMQ
        ctx = zmq.Context()
        self.zmq_rgb = ctx.socket(zmq.SUB)
        self.zmq_rgb.setsockopt(zmq.CONFLATE, 1)
        self.zmq_rgb.connect("tcp://127.0.0.1:5555")
        self.zmq_rgb.setsockopt_string(zmq.SUBSCRIBE, '')

        self.zmq_depth = ctx.socket(zmq.SUB)
        self.zmq_depth.setsockopt(zmq.CONFLATE, 1)
        self.zmq_depth.connect("tcp://127.0.0.1:5556")
        self.zmq_depth.setsockopt_string(zmq.SUBSCRIBE, '')

        # ── Estado
        self.estado             = "REPOSO_BRAZOS"
        self.t_estado           = time.time()
        self.t_last_cmd         = 0.0
        self.maniobra_iniciada  = False

        self.mem_z = 999.0
        self.mem_x_lat = 0.0
        self.mem_y_vert = 0.0
        self.mesa_z_pelvis = None
        self.caja_ancho_m = 0.20
        self.caja_alto_m = 0.20

        self.rest_targets_r, self.rest_targets_l = self._load_reposo_from_srdf()

        # ── Control
        self._rest_active = True
        self._rest_thread = threading.Thread(target=self._mantener_reposo, daemon=True)
        self._rest_thread.start()

        self.get_logger().info("🧠 MoveItBrain iniciado → REPOSO_BRAZOS")
        threading.Thread(target=self._check_moveit_ready, daemon=True).start()
        threading.Thread(target=self.vision_loop, daemon=True).start()

    # ══════════════════════════════════════════════════════════════════════════
    # EL PUENTE CONTROLADOR (MoveIt -> UDP)
    # ══════════════════════════════════════════════════════════════════════════
    def _state_cb_r(self, msg: JointTrajectoryControllerState):
        """Intercepta las órdenes de MoveIt para el brazo derecho y las guarda"""
        if not msg.reference.positions: return
        for i, name in enumerate(msg.joint_names):
            m_id = JOINT_MAP_R.get(name)
            if m_id: self.cmd_r[str(m_id)] = msg.reference.positions[i]
        self._enviar_estado_en_vivo()

    def _state_cb_l(self, msg: JointTrajectoryControllerState):
        """Intercepta las órdenes de MoveIt para el brazo izquierdo y las guarda"""
        if not msg.reference.positions: return
        for i, name in enumerate(msg.joint_names):
            m_id = JOINT_MAP_L.get(name)
            if m_id: self.cmd_l[str(m_id)] = msg.reference.positions[i]
        self._enviar_estado_en_vivo()

    def _enviar_estado_en_vivo(self):
        """Si la IA ha soltado el control de reposo, enviamos lo que diga MoveIt a MuJoCo"""
        if not self._rest_active:
            merged = {**self.cmd_r, **self.cmd_l}
            if merged:
                self.udp_arm.sendto(json.dumps(merged).encode(), self.ia_addr)

    def _load_reposo_from_srdf(self):
        self.get_logger().info("✅ Forzando pose de reposo SEGURA (sin colisiones).")
        rest_r = { 22: 0.20, 23: -0.20, 24: 0.00, 25: 0.40, 26: 0.00, 27: 0.00, 28: 0.00 }
        rest_l = { 15: 0.20, 16: -0.20, 17: 0.00, 18: 0.40, 19: 0.00, 20: 0.00, 21: 0.00 }
        return rest_r, rest_l

    def _check_moveit_ready(self):
        if self._action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().info("✅ Servidor MoveIt DISPONIBLE. (Puedes moverlo desde RViz)")

    def send_walk(self, cmd: str):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.2)
            s.connect(('127.0.0.1', 6000))
            s.sendall(cmd.encode())
            s.recv(64)
            s.close()
        except: pass

    def _mantener_reposo(self):
        combined_rest = {**self.rest_targets_r, **self.rest_targets_l}
        while True:
            if self._rest_active and combined_rest:
                payload = {str(k): float(v) for k, v in combined_rest.items()}
                self.udp_arm.sendto(json.dumps(payload).encode(), self.ia_addr)
            time.sleep(0.05)

    def _publicar_mesa_colision(self, table_z_pelvis: float, table_x: float):
        co = CollisionObject()
        co.header.frame_id = "pelvis"
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = "mesa"
        co.operation       = CollisionObject.ADD
        
        alto_pilar = 1.0
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.4, 0.4, alto_pilar])
        pose = Pose()
        pose.position.x = table_x + 0.10  
        pose.position.y = 0.0
        pose.position.z = table_z_pelvis - (alto_pilar / 2.0) - 0.01
        pose.orientation.w = 1.0
        
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects.append(co)
        self._scene_pub.publish(ps)

    def _eliminar_objeto_colision(self, obj_id: str):
        co = CollisionObject()
        co.header.frame_id = "pelvis"
        co.id = obj_id
        co.operation = CollisionObject.REMOVE
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects.append(co)
        self._scene_pub.publish(ps)

    def _limpiar_octomap(self):
        if self._clear_octomap.wait_for_service(timeout_sec=1.0):
            self._clear_octomap.call_async(Empty.Request())

    def _analizar_escena(self, frame: np.ndarray, depth: np.ndarray) -> bool:
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours: return False
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 400: return False

        x, y, w, h = cv2.boundingRect(c)
        u_c, v_c = int(x + w / 2), int(y + h / 2)
        z = depth[v_c, u_c]
        if np.isnan(z) or z < 0.05: return False

        self.caja_ancho_m = max(w * z / FOCAL_LENGTH, 0.05)
        self.caja_alto_m  = max(h * z / FOCAL_LENGTH, 0.05)
        
        v_tabla_min = min(y + h + 5,  IMG_H - 1)
        v_tabla_max = min(y + h + 35, IMG_H - 1)

        if v_tabla_min >= IMG_H:
            tabla_depth_samples = np.array([z])
        else:
            franja = depth[v_tabla_min:v_tabla_max, max(x-10, 0):min(x+w+10, IMG_W)]
            franja_valid = franja[(franja > 0.05) & (franja < 4.0)]
            tabla_depth_samples = franja_valid if franja_valid.size >= 5 else np.array([z])

        z_mesa_cam  = float(np.median(tabla_depth_samples))
        v_mesa_px   = v_tabla_min + (v_tabla_max - v_tabla_min) // 2
        y_vert_mesa = (v_mesa_px - CY) * z_mesa_cam / FOCAL_LENGTH
        mesa_3d     = cam_to_pelvis(z_mesa_cam, 0.0, y_vert_mesa)
        
        self.mesa_z_pelvis = mesa_3d[2]
        caja_3d = cam_to_pelvis(z, (u_c - CX) * z / FOCAL_LENGTH, (v_c - CY) * z / FOCAL_LENGTH)
        
        self._publicar_mesa_colision(self.mesa_z_pelvis, caja_3d[0])
        return True

    def _build_motion_request(self, group_name: str, tx: float, ty: float, tz: float, pos_tol: float = 0.06) -> MotionPlanRequest:
        end_effector = "right_rubber_hand" if "right" in group_name else "left_rubber_hand"
        req = MotionPlanRequest()
        req.group_name = group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 15
        req.start_state.is_diff = True

        pos = PositionConstraint()
        pos.header.frame_id = "pelvis"
        pos.link_name = end_effector
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
        target = Pose()
        target.position.x, target.position.y, target.position.z = tx, ty, tz
        pos.constraint_region.primitives.append(region)
        pos.constraint_region.primitive_poses.append(target)
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = "pelvis"
        ori.link_name = end_effector
        ori.orientation.w = 1.0
        ori.absolute_x_axis_tolerance = 1.0 
        ori.absolute_y_axis_tolerance = 1.0
        ori.absolute_z_axis_tolerance = 1.0
        ori.weight = 0.1

        c = Constraints()
        c.position_constraints.append(pos)
        c.orientation_constraints.append(ori)
        req.goal_constraints.append(c)
        return req

    def _ejecutar_maniobra(self):
        caja = cam_to_pelvis(self.mem_z, self.mem_x_lat, self.mem_y_vert)
        self.get_logger().info(f"🎯 Caja en pelvis: X={caja[0]:.3f}  Y={caja[1]:.3f}  Z={caja[2]:.3f}")

        half_w = self.caja_ancho_m / 2.0
        min_z = (self.mesa_z_pelvis + 0.04) if self.mesa_z_pelvis is not None else (caja[2] - self.caja_alto_m / 2.0 + 0.04)
        work_z = max(caja[2], min_z)

        # ── CORRECCIÓN GEOMÉTRICA FÍSICA ──
        # La muñeca está a ~10-12 cm por detrás de la punta de los dedos. 
        # Restamos a la X para que la mano no atraviese la caja frontalmente, 
        # y sumamos a la Y para que la muñeca quede fuera del volumen de la caja.
        OFFSET_MUNECA_X = 0.09 # La muñeca se queda 9 cm más cerca del robot
        OFFSET_MUNECA_Y = 0.06 # La muñeca se abre 6 cm más hacia los lados
        
        wp_x   = caja[0] - OFFSET_MUNECA_X
        wp_z   = work_z + 0.02 # Elevamos ligeramente las manos
        wp_y_r = caja[1] - half_w - OFFSET_MUNECA_Y
        wp_y_l = caja[1] + half_w + OFFSET_MUNECA_Y

        self.get_logger().info(f"🤜 Destino Derecho   → ({wp_x:.2f}, {wp_y_r:.2f}, {wp_z:.2f})")
        self.get_logger().info(f"🤛 Destino Izquierdo → ({wp_x:.2f}, {wp_y_l:.2f}, {wp_z:.2f})")

        # ── ENVIAMOS LA EJECUCIÓN NATIVA A MOVEIT ──
        goal_r = MoveGroup.Goal()
        goal_r.request = self._build_motion_request('right_arm', wp_x, wp_y_r, wp_z, 0.08)
        goal_r.planning_options.plan_only = False # ¡FALSO! Esto hace que MoveIt LO EJECUTE de verdad

        goal_l = MoveGroup.Goal()
        goal_l.request = self._build_motion_request('left_arm', wp_x, wp_y_l, wp_z, 0.08)
        goal_l.planning_options.plan_only = False

        # Desactivamos el reposo para que las callbacks del puente tomen el control
        self._rest_active = False
        time.sleep(0.1)

        self.get_logger().info("🚀 ¡Mandando orden de ejecución simultánea a MoveIt!")
        
        # Enviamos las órdenes de forma asíncrona para que MoveIt mueva los dos brazos a la vez
        future_r = self._action_client.send_goal_async(goal_r)
        future_l = self._action_client.send_goal_async(goal_l)

        # Esperamos a que los dos terminen
        while not (future_r.done() and future_l.done()):
            time.sleep(0.1)

        self.get_logger().info("🎉 ¡MISIÓN COMPLETADA! Abrazando la caja.")
        self.estado = "FINALIZADO"

    def vision_loop(self):
        last_frame = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        last_depth = np.ones((IMG_H, IMG_W), dtype=np.float32)

        while rclpy.ok():
            t0 = time.time()
            try:
                buf_rgb = self.zmq_rgb.recv(flags=zmq.NOBLOCK)
                decoded = cv2.imdecode(np.frombuffer(buf_rgb, np.uint8), 1)
                if decoded is not None: last_frame = decoded
            except: pass

            try:
                buf_depth = self.zmq_depth.recv(flags=zmq.NOBLOCK)
                last_depth = np.frombuffer(buf_depth, np.float32).reshape((IMG_H, IMG_W))
            except: pass

            frame = last_frame.copy()
            depth = last_depth

            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            box_detected = False
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 400:
                    x, y, w, h = cv2.boundingRect(c)
                    u_c, v_c = int(x + w / 2), int(y + h / 2)
                    z = depth[v_c, u_c]

                    if not np.isnan(z) and z > 0.05:
                        box_detected = True
                        self.mem_z = z
                        self.mem_x_lat = (u_c - CX) * z / FOCAL_LENGTH
                        self.mem_y_vert = (v_c - CY) * z / FOCAL_LENGTH
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            now = time.time()
            cv2.putText(frame, f"ESTADO: {self.estado}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            if self.estado == "REPOSO_BRAZOS":
                if now - self.t_estado > 2.0:
                    self.estado = "BUSCANDO"
                    self.t_estado = now

            elif self.estado == "BUSCANDO":
                if box_detected and self.mem_z < 3.0:
                    self.send_walk('stop')
                    self.estado = "CENTRANDO_ROTACION"
                    self.t_estado = now
                elif now - self.t_last_cmd > 0.15:
                    self.send_walk('q')
                    self.t_last_cmd = now

            elif self.estado == "CENTRANDO_ROTACION":
                if not box_detected:
                    self.estado = "BUSCANDO"
                elif abs(self.mem_x_lat) > 0.04:
                    if now - self.t_last_cmd > 0.15:
                        self.send_walk('e' if self.mem_x_lat > 0 else 'q')
                        self.t_last_cmd = now
                else:
                    self.send_walk('stop')
                    self.estado = "CAMINANDO_RECTO"

            elif self.estado == "CAMINANDO_RECTO":
                TARGET_DIST = 0.3
                if self.mem_z <= TARGET_DIST:
                    self.send_walk('stop')
                    self.estado = "AJUSTE_FINAL"
                    self.t_estado = now
                else:
                    if abs(self.mem_x_lat) > 0.12 and now - self.t_last_cmd > 0.20:
                        self.send_walk('d' if self.mem_x_lat > 0 else 'a')
                        self.t_last_cmd = now
                    elif now - self.t_last_cmd > 0.20:
                        self.send_walk('w')
                        self.t_last_cmd = now

            elif self.estado == "AJUSTE_FINAL":
                UMBRAL_LAT = 0.02
                if abs(self.mem_x_lat) > UMBRAL_LAT:
                    if now - self.t_last_cmd > 0.15:
                        self.send_walk('d' if self.mem_x_lat > 0 else 'a')
                        self.t_last_cmd = now
                else:
                    self.send_walk('stop')
                    self.estado = "ESTABILIZANDO"
                    self.t_estado = now

            elif self.estado == "ESTABILIZANDO":
                if now - self.t_estado > 2.5:
                    self.estado = "ANALIZANDO_ESCENA"
                    self.t_estado = now

            elif self.estado == "ANALIZANDO_ESCENA":
                if not self.maniobra_iniciada:
                    if abs(self.mem_x_lat) > 0.06:
                        self.estado = "AJUSTE_FINAL"
                        self.t_estado = now
                        continue

                    self._eliminar_objeto_colision("mesa")
                    time.sleep(0.1)
                    self._limpiar_octomap()
                    time.sleep(0.5)

                    if self._analizar_escena(frame, depth):
                        self.maniobra_iniciada = True
                        self.estado = "EJECUTANDO_MANIOBRA"
                        threading.Thread(target=self._ejecutar_maniobra, daemon=True).start()
                    else:
                        time.sleep(1.0)

            elif self.estado == "FINALIZADO":
                cv2.putText(frame, "MISION COMPLETADA", (140, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 128), 3)

            cv2.imshow("Cerebro G1 – MoveIt Brain (Lazo Cerrado)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            time.sleep(max(0.0, 0.033 - (time.time() - t0)))

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = MoveItBrain()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()
