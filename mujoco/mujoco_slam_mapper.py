#!/usr/bin/env python3
"""
mujoco_slam_mapper.py
=====================
Mapeador de rejilla de ocupación (Occupancy Grid) casero para ROS 2 Humble.

Suscribe:
    /scan   (sensor_msgs/LaserScan)   — escaneo 2D del LiDAR
    /odom   (nav_msgs/Odometry)       — odometría del robot

Publica:
    /map    (nav_msgs/OccupancyGrid)  — mapa de ocupación

Funciona con odometría como pose (sin scan matching).
En simulación con odom perfecto esto es equivalente a "mapping with known poses".

Uso:
    python3 mujoco_slam_mapper.py

    # En otra terminal:
    rviz2 -d rviz2/lidar_maze.rviz
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header


# ===========================================================================
# Configuración del mapa
# ===========================================================================
RESOLUTION    = 0.05     # Metros por celda (5 cm)
MAP_WIDTH_M   = 20.0     # Ancho del mapa en metros
MAP_HEIGHT_M  = 20.0     # Alto del mapa en metros
ORIGIN_X      = -10.0    # Esquina inferior-izquierda del mapa (metros)
ORIGIN_Y      = -10.0

MAP_WIDTH     = int(MAP_WIDTH_M / RESOLUTION)    # 400 celdas
MAP_HEIGHT    = int(MAP_HEIGHT_M / RESOLUTION)    # 400 celdas

# Log-odds para actualización bayesiana
L_FREE     = -0.4    # Log-odds: rayo pasa por aquí → libre
L_OCC      =  0.85   # Log-odds: rayo termina aquí → ocupado
L_PRIOR    =  0.0    # Prior (desconocido)
L_MIN      = -5.0    # Clamp mínimo
L_MAX      =  5.0    # Clamp máximo

# Publicación
MAP_PUBLISH_HZ = 2.0     # Frecuencia de publicación del mapa
MAP_FRAME      = "odom"   # Frame del mapa (coincide con odom ya que es ground truth)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


# ===========================================================================
# Bresenham line algorithm
# ===========================================================================
def bresenham(x0: int, y0: int, x1: int, y1: int):
    """
    Genera las celdas (x, y) en la línea desde (x0, y0) hasta (x1, y1).
    NO incluye el punto final (x1, y1) — ese se trata como ocupado aparte.
    """
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    err = dx - dy

    while True:
        if x0 == x1 and y0 == y1:
            break
        cells.append((x0, y0))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return cells


# ===========================================================================
# Nodo principal
# ===========================================================================
class OccupancyGridMapper(Node):

    def __init__(self):
        super().__init__("occupancy_grid_mapper")

        # --- Mapa (log-odds) ---
        self.log_odds = np.full(
            (MAP_HEIGHT, MAP_WIDTH), L_PRIOR, dtype=np.float32)

        # --- Última pose conocida ---
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._have_odom = False

        # --- Subscribers ---
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, SENSOR_QOS)
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, SENSOR_QOS)

        # --- Publisher del mapa ---
        self.map_pub = self.create_publisher(
            OccupancyGrid, "/map", 10)

        # --- Timer de publicación ---
        self._timer = self.create_timer(
            1.0 / MAP_PUBLISH_HZ, self._publish_map)

        self._scan_count = 0

        self.get_logger().info(
            f"OccupancyGridMapper arrancado.\n"
            f"  Mapa: {MAP_WIDTH}x{MAP_HEIGHT} celdas "
            f"({MAP_WIDTH_M}x{MAP_HEIGHT_M} m, res={RESOLUTION} m)\n"
            f"  Publica: /map a {MAP_PUBLISH_HZ} Hz"
        )

    # -------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        """Actualiza la pose del robot desde la odometría."""
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

        # Extraer yaw del quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self._have_odom = True

    # -------------------------------------------------------------------
    def _scan_cb(self, msg: LaserScan):
        """Integra un scan en el mapa de ocupación."""
        if not self._have_odom:
            return

        rx = self._robot_x
        ry = self._robot_y
        ryaw = self._robot_yaw

        # Celda del robot
        rx_cell = int((rx - ORIGIN_X) / RESOLUTION)
        ry_cell = int((ry - ORIGIN_Y) / RESOLUTION)

        angle = msg.angle_min + ryaw  # Añadir orientación del robot

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Punto de impacto en world
                hx = rx + r * math.cos(angle)
                hy = ry + r * math.sin(angle)

                # Celda del impacto
                hx_cell = int((hx - ORIGIN_X) / RESOLUTION)
                hy_cell = int((hy - ORIGIN_Y) / RESOLUTION)

                # Celdas libres a lo largo del rayo (Bresenham)
                free_cells = bresenham(rx_cell, ry_cell, hx_cell, hy_cell)
                for (cx, cy) in free_cells:
                    if 0 <= cx < MAP_WIDTH and 0 <= cy < MAP_HEIGHT:
                        self.log_odds[cy, cx] = max(
                            L_MIN, self.log_odds[cy, cx] + L_FREE)

                # Celda ocupada (punto de impacto)
                if 0 <= hx_cell < MAP_WIDTH and 0 <= hy_cell < MAP_HEIGHT:
                    self.log_odds[hy_cell, hx_cell] = min(
                        L_MAX, self.log_odds[hy_cell, hx_cell] + L_OCC)

            angle += msg.angle_increment

        self._scan_count += 1
        if self._scan_count % 50 == 0:
            occ = np.sum(self.log_odds > 0.5)
            free = np.sum(self.log_odds < -0.5)
            self.get_logger().info(
                f"Scans integrados: {self._scan_count} | "
                f"Ocupadas: {occ} | Libres: {free}")

    # -------------------------------------------------------------------
    def _publish_map(self):
        """Convierte log-odds a OccupancyGrid y publica."""
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = MAP_FRAME

        msg.info = MapMetaData()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = RESOLUTION
        msg.info.width = MAP_WIDTH
        msg.info.height = MAP_HEIGHT

        # Origen del mapa (esquina inferior-izquierda)
        msg.info.origin = Pose()
        msg.info.origin.position.x = ORIGIN_X
        msg.info.origin.position.y = ORIGIN_Y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convertir log-odds a probabilidad [0, 100] o -1 (desconocido)
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))   # Sigmoide → [0, 1]

        grid = np.full(self.log_odds.shape, -1, dtype=np.int8)
        # Libre: probabilidad < 0.35 → valor 0
        grid[prob < 0.35] = 0
        # Ocupado: probabilidad > 0.65 → valor 100
        grid[prob > 0.65] = 100
        # Entre 0.35 y 0.65 → desconocido (-1)

        msg.data = grid.ravel().tolist()

        self.map_pub.publish(msg)


# ===========================================================================
# main
# ===========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
