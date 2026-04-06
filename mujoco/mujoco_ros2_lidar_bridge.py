#!/usr/bin/env python3
"""
mujoco_ros2_lidar_bridge.py
===========================
Nodo ROS 2 que recibe el stream de LiDAR simulado desde MuJoCo (vía ZMQ)
y publica:

  Topics:
    /lidar/points   (sensor_msgs/msg/PointCloud2)  — nube de puntos 3D
    /scan           (sensor_msgs/msg/LaserScan)    — scan 2D (para SLAM)
    /odom           (nav_msgs/msg/Odometry)         — odometría del robot
    /tf             — transforms dinámicos
    /tf_static      — world → odom (identidad)

  TF publicados:
    world  →  odom       (identidad estática)
    odom   →  base_link  (pose del pelvis)
    base_link → lidar_link (pose relativa del LiDAR)

Protocolo ZMQ (puerto 5556, SUB):
  [magic: uint32][n_pts: uint32]
  [pelvis_pose: 7×float64  (x,y,z, qw,qx,qy,qz)]
  [lidar_pose:  7×float64  (x,y,z, qw,qx,qy,qz)]
  [timestamp:   float64]
  [points:      n_pts×3×float32  (XYZ en frame lidar_link)]

Uso:
  # Terminal 1 — arrancar simulación MuJoCo:
  cd mujoco/simulator && python3 unitree_mujoco.py

  # Terminal 2 — arrancar policy de locomoción:
  cd mujoco && python3 run_sim_ai_g1.py

  # Terminal 3 — arrancar este bridge:
  python3 mujoco/mujoco_ros2_lidar_bridge.py

  # Terminal 4 — visualizar:
  rviz2 -d rviz2/lidar_maze.rviz
"""

import struct
import sys

import numpy as np
import zmq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import math


# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------
ZMQ_HOST   = "localhost"
ZMQ_PORT   = 5556
ZMQ_MAGIC  = 0xDEAD1337

WORLD_FRAME = "world"
ODOM_FRAME  = "odom"
BASE_FRAME  = "base_link"
LIDAR_FRAME = "lidar_link"

LIDAR_TOPIC = "/lidar/points"
SCAN_TOPIC  = "/scan"
ODOM_TOPIC  = "/odom"

# Parámetros del LiDAR (deben coincidir con unitree_mujoco.py)
LIDAR_N_AZIMUTH = 360
LIDAR_MAX_RANGE = 12.0
LIDAR_MIN_RANGE = 0.15

# QoS: sensor data — best-effort, keep last 5
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


# ---------------------------------------------------------------------------
# Nodo principal
# ---------------------------------------------------------------------------
class MujocoLidarBridge(Node):

    def __init__(self):
        super().__init__("mujoco_lidar_bridge")

        # --- Publisher PointCloud2 ---
        self.pc_pub = self.create_publisher(
            PointCloud2, LIDAR_TOPIC, SENSOR_QOS)

        # --- Publisher LaserScan (para SLAM) ---
        self.scan_pub = self.create_publisher(
            LaserScan, SCAN_TOPIC, SENSOR_QOS)

        # --- Publisher Odometry ---
        self.odom_pub = self.create_publisher(
            Odometry, ODOM_TOPIC, SENSOR_QOS)

        # --- TF broadcaster ---
        self.tf_br = TransformBroadcaster(self)

        # --- Static TF: world → odom (identidad, ground truth) ---
        self.static_tf_br = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = WORLD_FRAME
        static_tf.child_frame_id = ODOM_FRAME
        static_tf.transform.rotation.w = 1.0
        self.static_tf_br.sendTransform(static_tf)

        # --- Para calcular velocidades ---
        self._prev_pelvis = None
        self._prev_time = None

        # --- ZMQ subscriber ---
        self._zmq_ctx  = zmq.Context()
        self._zmq_sock = self._zmq_ctx.socket(zmq.SUB)
        self._zmq_sock.connect(f"tcp://{ZMQ_HOST}:{ZMQ_PORT}")
        self._zmq_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._zmq_sock.setsockopt(zmq.RCVTIMEO, 100)   # 100 ms timeout

        # --- Timer de polling (20 Hz — más rápido que el LiDAR a 10 Hz) ---
        self._timer = self.create_timer(0.05, self._poll_and_publish)

        self.get_logger().info(
            f"MujocoLidarBridge arrancado.\n"
            f"  ZMQ: tcp://{ZMQ_HOST}:{ZMQ_PORT}\n"
            f"  Topics: {LIDAR_TOPIC}, {SCAN_TOPIC}, {ODOM_TOPIC}\n"
            f"  TF: {WORLD_FRAME} → {ODOM_FRAME} → {BASE_FRAME} → {LIDAR_FRAME}"
        )

    # -----------------------------------------------------------------------
    def _poll_and_publish(self):
        """Recibe un frame ZMQ (non-blocking) y publica ROS 2."""
        try:
            raw = self._zmq_sock.recv()
        except zmq.Again:
            return   # No hay datos nuevos todavía

        # --- Parsear cabecera ---
        offset = 0
        if len(raw) < 8:
            self.get_logger().warn("Mensaje ZMQ demasiado corto")
            return

        magic, n_pts = struct.unpack_from("=II", raw, offset)
        offset += 8

        if magic != ZMQ_MAGIC:
            self.get_logger().warn(
                f"Magic inválido: {magic:#010x} (esperado {ZMQ_MAGIC:#010x})")
            return

        # --- Pelvis pose (7 × float64) ---
        if len(raw) < offset + 7 * 8:
            return
        pelvis = np.frombuffer(raw, dtype=np.float64, count=7, offset=offset)
        offset += 7 * 8

        # --- Lidar pose (7 × float64) ---
        if len(raw) < offset + 7 * 8:
            return
        lidar = np.frombuffer(raw, dtype=np.float64, count=7, offset=offset)
        offset += 7 * 8

        # --- Timestamp (float64) ---
        if len(raw) < offset + 8:
            return
        # timestamp = struct.unpack_from("=d", raw, offset)[0]  # no usado por ahora
        offset += 8

        # --- Puntos (n_pts × 3 × float32) ---
        expected_bytes = n_pts * 3 * 4
        if len(raw) < offset + expected_bytes:
            self.get_logger().warn(
                f"Mensaje incompleto: esperados {expected_bytes} bytes de puntos, "
                f"disponibles {len(raw) - offset}")
            return

        if n_pts > 0:
            pts = np.frombuffer(
                raw, dtype=np.float32, count=n_pts * 3, offset=offset
            ).reshape(n_pts, 3)
        else:
            pts = np.zeros((0, 3), dtype=np.float32)

        # --- Stamp ROS 2 ---
        ros_stamp = self.get_clock().now().to_msg()
        now_sec = ros_stamp.sec + ros_stamp.nanosec * 1e-9

        # --- Calcular velocidades (diferencia finita) ---
        lin_vel = np.zeros(3)
        ang_vel = np.zeros(3)
        if self._prev_pelvis is not None and self._prev_time is not None:
            dt = now_sec - self._prev_time
            if dt > 1e-6:
                lin_vel = (pelvis[:3] - self._prev_pelvis[:3]) / dt
                q_prev_inv = self._quat_inv(self._prev_pelvis[3:7])
                q_delta = self._quat_mul(q_prev_inv, pelvis[3:7])
                ang_vel = 2.0 * q_delta[1:4] / dt

        self._prev_pelvis = pelvis.copy()
        self._prev_time = now_sec

        # --- Publicar TF: odom → base_link ---
        self.tf_br.sendTransform(
            self._make_tf(ros_stamp, ODOM_FRAME, BASE_FRAME, pelvis))

        # --- Publicar TF: base_link → lidar_link (transform relativo) ---
        rel_pose = self._relative_pose(pelvis, lidar)
        self.tf_br.sendTransform(
            self._make_tf(ros_stamp, BASE_FRAME, LIDAR_FRAME, rel_pose))

        # --- Publicar Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = ros_stamp
        odom_msg.header.frame_id = ODOM_FRAME
        odom_msg.child_frame_id = BASE_FRAME
        odom_msg.pose.pose.position.x = float(pelvis[0])
        odom_msg.pose.pose.position.y = float(pelvis[1])
        odom_msg.pose.pose.position.z = float(pelvis[2])
        odom_msg.pose.pose.orientation.w = float(pelvis[3])
        odom_msg.pose.pose.orientation.x = float(pelvis[4])
        odom_msg.pose.pose.orientation.y = float(pelvis[5])
        odom_msg.pose.pose.orientation.z = float(pelvis[6])
        odom_msg.twist.twist.linear.x = float(lin_vel[0])
        odom_msg.twist.twist.linear.y = float(lin_vel[1])
        odom_msg.twist.twist.linear.z = float(lin_vel[2])
        odom_msg.twist.twist.angular.x = float(ang_vel[0])
        odom_msg.twist.twist.angular.y = float(ang_vel[1])
        odom_msg.twist.twist.angular.z = float(ang_vel[2])
        self.odom_pub.publish(odom_msg)

        # --- Publicar PointCloud2 ---
        if n_pts > 0:
            self.pc_pub.publish(
                self._build_pointcloud2(ros_stamp, pts))

        # --- Publicar LaserScan (conversión de puntos 2D a rangos) ---
        self.scan_pub.publish(
            self._build_laserscan(ros_stamp, pts))

    # -----------------------------------------------------------------------
    @staticmethod
    def _make_tf(stamp, parent: str, child: str,
                 pose: np.ndarray) -> TransformStamped:
        """
        Construye un TransformStamped.
        pose = (x, y, z, qw, qx, qy, qz)  en float64.
        """
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = parent
        tf.child_frame_id  = child
        tf.transform.translation.x = float(pose[0])
        tf.transform.translation.y = float(pose[1])
        tf.transform.translation.z = float(pose[2])
        # MuJoCo: qw, qx, qy, qz → ROS geometry_msgs: x, y, z, w
        tf.transform.rotation.w = float(pose[3])
        tf.transform.rotation.x = float(pose[4])
        tf.transform.rotation.y = float(pose[5])
        tf.transform.rotation.z = float(pose[6])
        return tf

    # -----------------------------------------------------------------------
    @staticmethod
    def _quat_inv(q):
        """Inversa de un quaternion unitario (w,x,y,z) → (w,-x,-y,-z)."""
        return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)

    @staticmethod
    def _quat_mul(q1, q2):
        """Multiplicación de quaterniones (w,x,y,z)."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ], dtype=np.float64)

    @staticmethod
    def _quat_rotate(q, v):
        """Rota el vector v por el quaternion q (w,x,y,z)."""
        qv = np.array([0.0, v[0], v[1], v[2]], dtype=np.float64)
        qi = np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)
        # q * v_quat * q_inv
        r = MujocoLidarBridge._quat_mul(
            MujocoLidarBridge._quat_mul(q, qv), qi)
        return r[1:4]

    @staticmethod
    def _relative_pose(parent_pose, child_pose):
        """
        Calcula la pose relativa: T_parent_child = T_world_parent⁻¹ * T_world_child
        Ambos poses: (x,y,z, qw,qx,qy,qz)
        """
        p_parent = parent_pose[:3]
        q_parent = parent_pose[3:7]   # (qw, qx, qy, qz)

        p_child = child_pose[:3]
        q_child = child_pose[3:7]

        # Rotación relativa: q_rel = q_parent_inv * q_child
        q_parent_inv = MujocoLidarBridge._quat_inv(q_parent)
        q_rel = MujocoLidarBridge._quat_mul(q_parent_inv, q_child)

        # Posición relativa: p_rel = R_parent_inv * (p_child - p_parent)
        dp = p_child - p_parent
        p_rel = MujocoLidarBridge._quat_rotate(q_parent_inv, dp)

        return np.concatenate([p_rel, q_rel])

    # -----------------------------------------------------------------------
    @staticmethod
    def _build_laserscan(stamp, pts: np.ndarray) -> LaserScan:
        """
        Convierte puntos XYZ (en frame lidar_link) a LaserScan.
        Los puntos del LiDAR 2D están en el plano XY local.
        """
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = LIDAR_FRAME

        n = LIDAR_N_AZIMUTH
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi * (n - 1) / n
        msg.angle_increment = 2.0 * math.pi / n
        msg.time_increment = 0.0
        msg.scan_time = 0.1       # 10 Hz
        msg.range_min = float(LIDAR_MIN_RANGE)
        msg.range_max = float(LIDAR_MAX_RANGE)

        # Inicializar todos los rangos a inf (sin detección)
        ranges = [float('inf')] * n

        if len(pts) > 0:
            # Calcular ángulo y rango de cada punto
            angles = np.arctan2(pts[:, 1], pts[:, 0])   # [-π, π]
            angles = angles % (2.0 * math.pi)            # [0, 2π)
            dists = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)

            # Asignar cada punto a su bin angular
            bins = np.round(angles / msg.angle_increment).astype(int) % n

            for i in range(len(pts)):
                b = bins[i]
                d = float(dists[i])
                if LIDAR_MIN_RANGE < d < LIDAR_MAX_RANGE:
                    # Quedarse con el rango más cercano por bin
                    if d < ranges[b]:
                        ranges[b] = d

        msg.ranges = ranges
        msg.intensities = []
        return msg

    # -----------------------------------------------------------------------
    @staticmethod
    def _build_pointcloud2(stamp, pts: np.ndarray) -> PointCloud2:
        """
        Construye un PointCloud2 desde un array (N, 3) float32.
        Frame: lidar_link.
        """
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp    = stamp
        msg.header.frame_id = LIDAR_FRAME

        msg.height = 1
        msg.width  = len(pts)

        msg.fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 12          # 3 × 4 bytes
        msg.row_step     = msg.point_step * msg.width
        msg.data         = pts.astype(np.float32).tobytes()
        msg.is_dense     = True

        return msg

    # -----------------------------------------------------------------------
    def destroy_node(self):
        self._zmq_sock.close()
        self._zmq_ctx.term()
        super().destroy_node()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MujocoLidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
