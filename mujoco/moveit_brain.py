"""
moveit_brain.py  –  Cerebro completo para el Unitree G1 en MuJoCo.

Máquina de estados:
  REPOSO_BRAZOS        → Lleva el brazo derecho a posición replegada (UDP directo)
  BUSCANDO             → Rota buscando la caja verde
  CENTRANDO_ROTACION   → Gira sobre el sitio hasta centrar la caja en X
  CAMINANDO_RECTO      → Avanza; corrige deriva lateral en tiempo real
  AJUSTE_FINAL         → Recentrado fino (strafe) cuando ya está a distancia
  ESTABILIZANDO        → Pausa de 2 s para que las físicas de MuJoCo se asienten
  ANALIZANDO_ESCENA    → Estima caja + mesa con depth; publica CollisionObject
  EJECUTANDO_MANIOBRA  → Streaming de trayectoria MoveIt en 2 waypoints
  FINALIZADO           → Misión completada
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

from std_srvs.srv import Empty
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped

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

# ───────────────────────────────────────────────────────────── CÁMARA
FOCAL_LENGTH = 460.0        # píxeles  (coincide con perception_bridge)
CX, CY      = 320.0, 240.0  # centro óptico
IMG_W, IMG_H = 640, 480

# Offset físico cámara → pelvis (ajustar según el XML del G1)
CAM_OFFSET_X =  0.06   # cámara 6 cm por delante de la pelvis
CAM_OFFSET_Y =  0.00
CAM_OFFSET_Z =  0.15   # cámara 15 cm por encima de la pelvis

# ─────────────────────────────────────────────── POSICIÓN DE REPOSO BRAZO DERECHO
# Valores en radianes – brazo doblado, pegado al cuerpo, sin molestar al caminar
REST_TARGETS = {
    22: 0.30,   # right_shoulder_pitch  – leve hacia adelante
    23: -0.50,  # right_shoulder_roll   – brazo hacia el cuerpo
    24:  0.00,  # right_shoulder_yaw
    25:  1.20,  # right_elbow           – codo doblado (mano cerca del pecho)
    26:  0.00,  # right_wrist_roll
    27:  0.00,  # right_wrist_pitch
    28:  0.00,  # right_wrist_yaw
}

# Mapeo nombre → ID motor (para MoveIt → UDP)
JOINT_MAP = {
    "right_shoulder_pitch_joint": 22,
    "right_shoulder_roll_joint":  23,
    "right_shoulder_yaw_joint":   24,
    "right_elbow_joint":          25,
    "right_wrist_roll_joint":     26,
    "right_wrist_pitch_joint":    27,
    "right_wrist_yaw_joint":      28,
}


# ──────────────────────────────────────────────────────────────────────────────
def cam_to_pelvis(z_depth, x_lat, y_vert):
    """Proyección de coordenadas cámara → frame pelvis del robot."""
    x = CAM_OFFSET_X + z_depth
    y = CAM_OFFSET_Y - x_lat
    z = CAM_OFFSET_Z - y_vert
    return np.array([x, y, z], dtype=np.float64)


# ──────────────────────────────────────────────────────────────────────────────
class MoveItBrain(Node):

    def __init__(self):
        super().__init__('moveit_brain')

        # ── Clientes ROS 2 ──────────────────────────────────────────────────
        self._action_client    = ActionClient(self, MoveGroup, 'move_action')
        self._clear_octomap    = self.create_client(Empty, '/clear_octomap')
        self._scene_pub        = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # ── Sockets ─────────────────────────────────────────────────────────
        self.udp_arm   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ia_addr   = ('127.0.0.1', 9876)

        # ── ZMQ: vídeo RGB y depth de MuJoCo ────────────────────────────────
        ctx = zmq.Context()
        self.zmq_rgb = ctx.socket(zmq.SUB)
        self.zmq_rgb.setsockopt(zmq.CONFLATE, 1)
        self.zmq_rgb.connect("tcp://127.0.0.1:5555")
        self.zmq_rgb.setsockopt_string(zmq.SUBSCRIBE, '')

        self.zmq_depth = ctx.socket(zmq.SUB)
        self.zmq_depth.setsockopt(zmq.CONFLATE, 1)
        self.zmq_depth.connect("tcp://127.0.0.1:5556")
        self.zmq_depth.setsockopt_string(zmq.SUBSCRIBE, '')

        # ── Estado de la máquina ─────────────────────────────────────────────
        self.estado        = "REPOSO_BRAZOS"
        self.t_estado      = time.time()
        self.t_last_cmd    = 0.0
        self.maniobra_iniciada = False

        # ── Memoria de percepción ────────────────────────────────────────────
        self.mem_z      = 999.0   # profundidad caja (m)
        self.mem_x_lat  = 0.0    # lateral en cámara (m)
        self.mem_y_vert = 0.0    # vertical en cámara (m)
        self.mem_w_px   = 0      # ancho caja en píxeles
        self.mem_h_px   = 0      # alto caja en píxeles
        self.mem_box_bottom_v = CY  # fila de la base de la caja en imagen

        # Estimación de la mesa
        self.mesa_z_pelvis  = None   # altura superfice mesa en frame pelvis
        self.caja_ancho_m   = 0.20   # estimado en tiempo real
        self.caja_alto_m    = 0.20

        # ── Hilo de mantenimiento del brazo en reposo ────────────────────────
        self._rest_active = True
        self._rest_thread = threading.Thread(target=self._mantener_reposo, daemon=True)
        self._rest_thread.start()

        self.get_logger().info("🧠 MoveItBrain iniciado → REPOSO_BRAZOS")
        self.get_logger().info("⏳ Comprobando disponibilidad del servidor MoveIt…")
        threading.Thread(target=self._check_moveit_ready, daemon=True).start()
        threading.Thread(target=self.vision_loop, daemon=True).start()

    def _check_moveit_ready(self):
        """Hilo de diagnóstico: informa cuando MoveIt está disponible."""
        available = self._action_client.wait_for_server(timeout_sec=30.0)
        if available:
            self.get_logger().info("✅ Servidor MoveIt (move_action) DISPONIBLE.")
        else:
            self.get_logger().error(
                "🚨 MoveIt NO disponible tras 30 s. "
                "Comprueba que g1_moveit_config demo.launch.py está corriendo "
                "y que run_sim_ai_g1.py publica /joint_states correctamente."
            )

    # ══════════════════════════════════════════════════════════════════════════
    # CONTROL DE LOCOMOCIÓN (TCP → run_sim_ai_g1)
    # ══════════════════════════════════════════════════════════════════════════

    def send_walk(self, cmd: str):
        """Envía comando de locomoción al servidor TCP del run_sim_ai_g1."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.2)
            s.connect(('127.0.0.1', 6000))
            s.sendall(cmd.encode())
            s.recv(64)
            s.close()
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    # CONTROL DEL BRAZO (UDP → run_sim_ai_g1)
    # ══════════════════════════════════════════════════════════════════════════

    def _enviar_arm_targets(self, targets: dict):
        """Envía {motor_id: posicion_rad} por UDP."""
        payload = {str(k): float(v) for k, v in targets.items()}
        self.udp_arm.sendto(json.dumps(payload).encode(), self.ia_addr)

    def _mantener_reposo(self):
        """
        Hilo de fondo: mientras _rest_active=True, mantiene el brazo derecho
        en la posición replegada enviando UDP a 20 Hz.
        Se desactiva justo antes de hacer streaming de MoveIt.
        """
        while True:
            if self._rest_active:
                self._enviar_arm_targets(REST_TARGETS)
            time.sleep(0.05)

    # ══════════════════════════════════════════════════════════════════════════
    # PLANNING SCENE – MESA COMO OBJETO DE COLISIÓN
    # ══════════════════════════════════════════════════════════════════════════

    def _publicar_mesa_colision(self, table_z_pelvis: float, table_x: float):
        """
        Añade una caja plana a la Planning Scene de MoveIt representando la mesa.
        Se sitúa en el frame 'pelvis' del robot.
        """
        co = CollisionObject()
        co.header.frame_id = "pelvis"
        co.header.stamp    = self.get_clock().now().to_msg()
        co.id              = "mesa"
        co.operation       = CollisionObject.ADD

        # Caja grande y plana (2.5 m × 1.5 m × 3 cm)
        prim = SolidPrimitive()
        prim.type       = SolidPrimitive.BOX
        prim.dimensions = [2.5, 1.5, 0.03]

        pose = Pose()
        pose.position.x = table_x          # aproximadamente donde está la mesa
        pose.position.y = 0.0
        pose.position.z = table_z_pelvis   # altura superficie
        pose.orientation.w = 1.0

        co.primitives.append(prim)
        co.primitive_poses.append(pose)

        ps        = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)
        self._scene_pub.publish(ps)
        self.get_logger().info(f"🏗️  Mesa añadida como colisión en Z={table_z_pelvis:.3f} m")

    def _eliminar_objeto_colision(self, obj_id: str):
        """Elimina un objeto de la Planning Scene por ID."""
        co = CollisionObject()
        co.header.frame_id = "pelvis"
        co.id              = obj_id
        co.operation       = CollisionObject.REMOVE
        ps        = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)
        self._scene_pub.publish(ps)

    def _limpiar_octomap(self):
        if self._clear_octomap.wait_for_service(timeout_sec=1.0):
            self._clear_octomap.call_async(Empty.Request())
            self.get_logger().info("🧹 Octomap limpiado.")

    # ══════════════════════════════════════════════════════════════════════════
    # ANÁLISIS DE ESCENA – ESTIMACIÓN DE MESA Y CAJA
    # ══════════════════════════════════════════════════════════════════════════

    def _analizar_escena(self, frame: np.ndarray, depth: np.ndarray):
        """
        A partir del último frame RGB + depth cuando el robot está parado:
        1. Detecta la caja verde y estima sus dimensiones en metros.
        2. Muestrea profundidad por DEBAJO de la caja para estimar la mesa.
        3. Publica la mesa como CollisionObject en MoveIt.

        Devuelve True si el análisis fue exitoso.
        """
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 400:
            return False

        x, y, w, h = cv2.boundingRect(c)
        u_c = int(x + w / 2)
        v_c = int(y + h / 2)

        z = depth[v_c, u_c]
        if np.isnan(z) or z < 0.05:
            return False

        # ── Dimensiones reales de la caja ────────────────────────────────────
        self.caja_ancho_m = max(w * z / FOCAL_LENGTH, 0.05)
        self.caja_alto_m  = max(h * z / FOCAL_LENGTH, 0.05)
        self.get_logger().info(
            f"📦 Caja estimada: {self.caja_ancho_m*100:.1f} cm × {self.caja_alto_m*100:.1f} cm")

        # ── Muestreo de la superficie de la mesa ─────────────────────────────
        # Muestreamos en una franja justo debajo del borde inferior de la caja
        v_tabla_min = min(y + h + 5,  IMG_H - 1)
        v_tabla_max = min(y + h + 35, IMG_H - 1)

        if v_tabla_min >= IMG_H:
            # La caja ocupa hasta el borde inferior: inferimos altura mesa
            tabla_depth_samples = np.array([z])
        else:
            # Franja horizontal debajo de la caja
            franja = depth[v_tabla_min:v_tabla_max, max(x-10, 0):min(x+w+10, IMG_W)]
            franja_valid = franja[(franja > 0.05) & (franja < 4.0)]
            if franja_valid.size < 5:
                tabla_depth_samples = np.array([z])
            else:
                tabla_depth_samples = franja_valid

        z_mesa_cam   = float(np.median(tabla_depth_samples))
        v_mesa_px    = v_tabla_min + (v_tabla_max - v_tabla_min) // 2
        y_vert_mesa  = (v_mesa_px - CY) * z_mesa_cam / FOCAL_LENGTH

        mesa_3d = cam_to_pelvis(z_mesa_cam, 0.0, y_vert_mesa)
        self.mesa_z_pelvis = mesa_3d[2]

        self.get_logger().info(
            f"🪑 Mesa detectada: Z={self.mesa_z_pelvis:.3f} m en frame pelvis")

        # Publicar como CollisionObject
        caja_3d = cam_to_pelvis(z, (u_c - CX) * z / FOCAL_LENGTH,
                                   (v_c - CY) * z / FOCAL_LENGTH)
        self._publicar_mesa_colision(self.mesa_z_pelvis, caja_3d[0])
        return True

    # ══════════════════════════════════════════════════════════════════════════
    # MOVEIT – PLANIFICACIÓN Y EJECUCIÓN
    # ══════════════════════════════════════════════════════════════════════════

    def _build_motion_request(self, tx: float, ty: float, tz: float,
                               pos_tol: float = 0.06) -> MotionPlanRequest:
        """Construye un MotionPlanRequest para el grupo right_arm en frame pelvis."""
        req                       = MotionPlanRequest()
        req.group_name            = 'right_arm'
        req.allowed_planning_time = 8.0
        req.num_planning_attempts = 30
        req.start_state.is_diff   = True

        # Restricción de posición (esfera de tolerancia)
        pos              = PositionConstraint()
        pos.header.frame_id = "pelvis"
        pos.link_name    = "right_rubber_hand"

        region = SolidPrimitive()
        region.type       = SolidPrimitive.SPHERE
        region.dimensions = [pos_tol]

        target = Pose()
        target.position.x = tx
        target.position.y = ty
        target.position.z = tz

        pos.constraint_region.primitives.append(region)
        pos.constraint_region.primitive_poses.append(target)
        pos.weight = 1.0

        # Restricción de orientación (muñeca recta; tolerancias amplias)
        ori                          = OrientationConstraint()
        ori.header.frame_id          = "pelvis"
        ori.link_name                = "right_rubber_hand"
        ori.orientation.w            = 1.0
        ori.absolute_x_axis_tolerance = 0.6
        ori.absolute_y_axis_tolerance = 0.6
        ori.absolute_z_axis_tolerance = 0.6
        ori.weight                   = 0.8

        c = Constraints()
        c.position_constraints.append(pos)
        c.orientation_constraints.append(ori)
        req.goal_constraints.append(c)
        return req

    def _planificar(self, x: float, y: float, z: float, pos_tol: float = 0.07):
        """
        Llama a MoveIt y devuelve la trayectoria articular, o None si falla.
        Incluye: timeout de 20 s, detección de rechazo y log del error_code.
        """
        TIMEOUT_ACCEPT = 10.0   # segundos para que el servidor acepte el goal
        TIMEOUT_PLAN   = 20.0   # segundos para que devuelva el resultado

        # ── 1. Comprobar que el servidor está vivo ───────────────────────────
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ MoveIt action server NO disponible (timeout 5 s).")
            return None

        try:
            goal              = MoveGroup.Goal()
            goal.request      = self._build_motion_request(x, y, z, pos_tol)
            goal.planning_options.plan_only = True

            future = self._action_client.send_goal_async(goal)

            # ── 2. Esperar aceptación ────────────────────────────────────────
            t0 = time.time()
            while not future.done():
                if time.time() - t0 > TIMEOUT_ACCEPT:
                    self.get_logger().error("⏰ Timeout esperando ACEPTACIÓN del goal.")
                    return None
                time.sleep(0.05)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(
                    "❌ Goal RECHAZADO por MoveIt (not accepted). "
                    "Comprueba: nombre del grupo, links, TF y joint_states.")
                return None

            self.get_logger().info(f"✅ Goal aceptado. Planificando hacia ({x:.2f},{y:.2f},{z:.2f})…")

            # ── 3. Esperar resultado ─────────────────────────────────────────
            res_future = goal_handle.get_result_async()
            t0 = time.time()
            while not res_future.done():
                if time.time() - t0 > TIMEOUT_PLAN:
                    self.get_logger().error("⏰ Timeout esperando RESULTADO del planner.")
                    return None
                time.sleep(0.05)

            result = res_future.result().result
            error_code = result.error_code.val   # MoveItErrorCodes

            if error_code != 1:   # 1 = SUCCESS
                # Tabla de códigos de error frecuentes de MoveIt
                CODES = {
                    -1: "FAILURE",  -2: "PLANNING_FAILED", -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                    -5: "CONTROL_FAILED", -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                    -7: "TIMED_OUT", -8: "PREEMPTED", 99999: "INVALID_MOTION_PLAN",
                }
                nombre = CODES.get(error_code, f"código {error_code}")
                self.get_logger().error(f"❌ MoveIt devolvió error: {nombre}")
                return None

            traj = result.planned_trajectory.joint_trajectory
            if not traj or not traj.points:
                self.get_logger().error("❌ Trayectoria vacía recibida de MoveIt.")
                return None

            self.get_logger().info(f"📐 Trayectoria OK: {len(traj.points)} puntos.")
            return traj

        except Exception as e:
            self.get_logger().error(f"❌ Excepción en _planificar: {e}")
            return None

    def _stream_trayectoria(self, traj, speed_scale: float = 2.5):
        """
        Envía la trayectoria respetando los tiempos reales de MoveIt escalados
        por speed_scale (>1 = más lento). Mínimo 60 ms entre puntos.
        """
        self.get_logger().info(
            f"📡 Streaming {len(traj.points)} puntos (×{speed_scale:.1f} velocidad)…")
        t_prev = 0.0
        for pt in traj.points:
            payload = {}
            for i, name in enumerate(traj.joint_names):
                if name in JOINT_MAP:
                    payload[str(JOINT_MAP[name])] = float(pt.positions[i])
            self.udp_arm.sendto(json.dumps(payload).encode(), self.ia_addr)

            t_cur = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt    = max((t_cur - t_prev) * speed_scale, 0.06)
            t_prev = t_cur
            time.sleep(dt)

    # ══════════════════════════════════════════════════════════════════════════
    # MANIOBRA DE DOS WAYPOINTS
    # ══════════════════════════════════════════════════════════════════════════

    def _ejecutar_maniobra(self):
        """
        Dos waypoints con enfoque LATERAL (no por encima):
          1. Waypoint lateral: misma Z que el destino, 20 cm más a la derecha.
             El brazo se aproxima horizontalmente → sin poses raras.
          2. Waypoint final: palma apoyada en la cara lateral derecha de la caja.
        Reintenta hasta MAX_INTENTOS con tolerancia de posición creciente.
        """
        MAX_INTENTOS = 3
        caja = cam_to_pelvis(self.mem_z, self.mem_x_lat, self.mem_y_vert)
        self.get_logger().info(
            f"🎯 Caja en pelvis: X={caja[0]:.3f}  Y={caja[1]:.3f}  Z={caja[2]:.3f}")

        # ── Geometría compartida ─────────────────────────────────────────────
        half_w  = self.caja_ancho_m / 2.0
        min_z   = (self.mesa_z_pelvis + 0.04) if self.mesa_z_pelvis is not None \
                  else (caja[2] - self.caja_alto_m / 2.0 + 0.04)
        # Z de trabajo: altura del centro de la caja, nunca por debajo de la mesa
        work_z  = max(caja[2], min_z)

        # ── Waypoint 1: aproximación LATERAL ────────────────────────────────
        # Mismo X y Z que el destino final; 20 cm más al exterior en Y.
        # El brazo viaja en horizontal desde la posición de reposo → sin arco
        # vertical sobre la caja y sin poses antinaturales.
        lat_x = caja[0]
        lat_y = caja[1] - half_w - 0.20   # 20 cm al exterior de la cara
        lat_z = work_z

        self.get_logger().info(
            f"➡️  Waypoint lateral → ({lat_x:.2f}, {lat_y:.2f}, {lat_z:.2f})")

        traj1 = None
        for intento in range(1, MAX_INTENTOS + 1):
            self.get_logger().info(f"  Intento {intento}/{MAX_INTENTOS} waypoint lateral…")
            traj1 = self._planificar(lat_x, lat_y, lat_z,
                                     pos_tol=0.07 + (intento - 1) * 0.03)
            if traj1 is not None:
                break
            time.sleep(1.0)

        if traj1 is None:
            self.get_logger().error("❌ Waypoint lateral fallido tras todos los intentos.")
            self.estado = "FINALIZADO"
            return

        # Desactivar reposo y hacer streaming
        self._rest_active = False
        time.sleep(0.15)
        self._stream_trayectoria(traj1)
        time.sleep(0.5)    # pausa antes de la entrada final

        # ── Waypoint 2: cara lateral derecha ────────────────────────────────
        final_x = caja[0]
        final_y = caja[1] - half_w - 0.01   # 1 cm más allá de la cara
        final_z = work_z

        self.get_logger().info(
            f"🤜 Waypoint final → ({final_x:.2f}, {final_y:.2f}, {final_z:.2f})")

        traj2 = None
        for intento in range(1, MAX_INTENTOS + 1):
            self.get_logger().info(f"  Intento {intento}/{MAX_INTENTOS} waypoint final…")
            traj2 = self._planificar(final_x, final_y, final_z,
                                     pos_tol=0.06 + (intento - 1) * 0.03)
            if traj2 is not None:
                break
            time.sleep(1.0)

        if traj2 is None:
            self.get_logger().error("❌ Waypoint final fallido tras todos los intentos.")
            self.estado = "FINALIZADO"
            return

        self._stream_trayectoria(traj2)
        self.get_logger().info("🎉 ¡MISIÓN COMPLETADA! Palma sobre la cara lateral.")
        self.estado = "FINALIZADO"

    # ══════════════════════════════════════════════════════════════════════════
    # BUCLE DE VISIÓN Y MÁQUINA DE ESTADOS
    # ══════════════════════════════════════════════════════════════════════════

    def vision_loop(self):
        """Bucle principal a ~30 Hz: percepción + lógica de estados."""

        last_frame = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)  # frame de respaldo
        last_depth = np.ones((IMG_H, IMG_W), dtype=np.float32)

        while rclpy.ok():
            t0 = time.time()

            # ── Capturar frames (sin bloquear; usar último válido si no hay) ─
            try:
                buf_rgb = self.zmq_rgb.recv(flags=zmq.NOBLOCK)
                decoded = cv2.imdecode(np.frombuffer(buf_rgb, np.uint8), 1)
                if decoded is not None:
                    last_frame = decoded
            except zmq.Again:
                pass
            except Exception:
                pass

            try:
                buf_depth = self.zmq_depth.recv(flags=zmq.NOBLOCK)
                last_depth = np.frombuffer(buf_depth, np.float32).reshape((IMG_H, IMG_W))
            except zmq.Again:
                pass
            except Exception:
                pass

            frame = last_frame.copy()
            depth = last_depth

            # ── Detección de la caja verde ──────────────────────────────────
            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            box_detected = False
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 400:
                    x, y, w, h = cv2.boundingRect(c)
                    u_c = int(x + w / 2)
                    v_c = int(y + h / 2)
                    z   = depth[v_c, u_c]

                    if not np.isnan(z) and z > 0.05:
                        box_detected          = True
                        self.mem_z            = z
                        self.mem_x_lat        = (u_c - CX) * z / FOCAL_LENGTH
                        self.mem_y_vert       = (v_c - CY) * z / FOCAL_LENGTH
                        self.mem_w_px         = w
                        self.mem_h_px         = h
                        self.mem_box_bottom_v = y + h

                        # Overlay en la imagen
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.putText(frame, f"{z:.2f}m | lat:{self.mem_x_lat:.2f}",
                                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 0), 2)

            now = time.time()

            # ═══════════════════════════════════════════════════════════════
            # MÁQUINA DE ESTADOS
            # ═══════════════════════════════════════════════════════════════

            # Overlay del estado
            cv2.putText(frame, f"ESTADO: {self.estado}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            # ── REPOSO_BRAZOS ───────────────────────────────────────────────
            if self.estado == "REPOSO_BRAZOS":
                # Esperamos 2 s mientras el hilo de reposo posiciona el brazo
                if now - self.t_estado > 2.0:
                    self.get_logger().info("✅ Brazo en reposo. Iniciando búsqueda.")
                    self.estado   = "BUSCANDO"
                    self.t_estado = now

            # ── BUSCANDO ────────────────────────────────────────────────────
            elif self.estado == "BUSCANDO":
                if box_detected and self.mem_z < 3.0:
                    self.send_walk('stop')
                    self.estado   = "CENTRANDO_ROTACION"
                    self.t_estado = now
                elif now - self.t_last_cmd > 0.15:
                    self.send_walk('q')     # gira a la izquierda
                    self.t_last_cmd = now

            # ── CENTRANDO_ROTACION ──────────────────────────────────────────
            elif self.estado == "CENTRANDO_ROTACION":
                if not box_detected:
                    # Caja perdida → volver a buscar
                    self.estado   = "BUSCANDO"
                    self.t_estado = now
                elif abs(self.mem_x_lat) > 0.04:
                    if now - self.t_last_cmd > 0.15:
                        cmd = 'e' if self.mem_x_lat > 0 else 'q'
                        self.send_walk(cmd)
                        self.t_last_cmd = now
                else:
                    self.send_walk('stop')
                    self.get_logger().info("🎯 Centrado en rotación. Avanzando...")
                    self.estado   = "CAMINANDO_RECTO"
                    self.t_estado = now

            # ── CAMINANDO_RECTO ─────────────────────────────────────────────
            elif self.estado == "CAMINANDO_RECTO":
                # Distancia objetivo: 40–45 cm del frente de la caja
                TARGET_DIST = 0.42

                if self.mem_z <= TARGET_DIST:
                    self.send_walk('stop')
                    self.get_logger().info("📍 Distancia alcanzada. Ajuste fino...")
                    self.estado   = "AJUSTE_FINAL"
                    self.t_estado = now
                else:
                    # Corrección lateral en tiempo real mientras camina
                    if abs(self.mem_x_lat) > 0.12 and now - self.t_last_cmd > 0.20:
                        # Deriva lateral grande: strafe primero
                        cmd = 'd' if self.mem_x_lat > 0 else 'a'
                        self.send_walk(cmd)
                        self.t_last_cmd = now
                    elif now - self.t_last_cmd > 0.20:
                        self.send_walk('w')
                        self.t_last_cmd = now

            # ── AJUSTE_FINAL ────────────────────────────────────────────────
            elif self.estado == "AJUSTE_FINAL":
                # Strafe fino para centrar la caja en X
                UMBRAL_LAT = 0.025
                if abs(self.mem_x_lat) > UMBRAL_LAT:
                    if now - self.t_last_cmd > 0.15:
                        cmd = 'd' if self.mem_x_lat > UMBRAL_LAT else 'a'
                        self.send_walk(cmd)
                        self.t_last_cmd = now
                else:
                    self.send_walk('stop')
                    self.get_logger().info("✨ Alineado. Estabilizando...")
                    self.estado   = "ESTABILIZANDO"
                    self.t_estado = now

            # ── ESTABILIZANDO ───────────────────────────────────────────────
            elif self.estado == "ESTABILIZANDO":
                if now - self.t_estado > 2.5:
                    self.get_logger().info("🔬 Analizando escena con depth...")
                    self.estado   = "ANALIZANDO_ESCENA"
                    self.t_estado = now

            # ── ANALIZANDO_ESCENA ───────────────────────────────────────────
            elif self.estado == "ANALIZANDO_ESCENA":
                if not self.maniobra_iniciada:
                    # Limpiar artefactos de la escena anterior
                    self._eliminar_objeto_colision("mesa")
                    time.sleep(0.1)
                    self._limpiar_octomap()
                    time.sleep(0.5)

                    ok = self._analizar_escena(frame, depth)
                    if ok:
                        self.maniobra_iniciada = True
                        self.estado = "EJECUTANDO_MANIOBRA"
                        threading.Thread(target=self._ejecutar_maniobra, daemon=True).start()
                    else:
                        self.get_logger().warn("⚠️  Análisis fallido. Reintentando en 1 s…")
                        time.sleep(1.0)

            # ── EJECUTANDO_MANIOBRA ─────────────────────────────────────────
            elif self.estado == "EJECUTANDO_MANIOBRA":
                pass  # el hilo de _ejecutar_maniobra cambia el estado

            # ── FINALIZADO ──────────────────────────────────────────────────
            elif self.estado == "FINALIZADO":
                cv2.putText(frame, "MISION COMPLETADA", (140, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 128), 3)

            # ── Visualización ───────────────────────────────────────────────
            cv2.imshow("Cerebro G1 – MoveIt Brain", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(max(0.0, 0.033 - (time.time() - t0)))   # ~30 Hz

        cv2.destroyAllWindows()


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MoveItBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
