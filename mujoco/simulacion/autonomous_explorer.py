#!/usr/bin/env python3
"""
wall_follower.py  —  G1 Wall Follower
======================================
Sigue la pared del lado derecho (SIDE="right") o izquierdo (SIDE="left").

FASES:
  1. APPROACH : avanza en línea recta hasta que la pared lateral
                esté a WALL_IDEAL metros. No gira, va recto.
  2. FOLLOW   : avanza pegado a la pared. Corrección con df/db.
  3. CORNER_IN: obstáculo al frente → gira lejos hasta despejarlo.
  4. DONE     : vuelta completa, para.

MAPA: publica /map en RViz con SLAM propio (igual que mujoco_slam_mapper).
"""

import math, socket, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

try:
    import zmq, cv2
    _CAM = True
except ImportError:
    _CAM = False

# ══════════════════════════════════════════════════════════════════════════════
# PARÁMETROS
# ══════════════════════════════════════════════════════════════════════════════
TCP_HOST   = "127.0.0.1"
TCP_PORT   = 6000
CAM_PORT   = 5555

SIDE       = "right"   # "right" o "left"

WALL_IDEAL = 0.62      # distancia objetivo a la pared (m)
WALL_CLOSE = 0.44      # demasiado cerca → alejarse
WALL_FAR   = 0.95      # demasiado lejos → acercarse

FRONT_STOP = 0.42      # emergencia absoluta
FRONT_TURN = 0.75      # preparar giro de esquina interior

MIN_TRAVEL = 5.0       # m mínimos antes de buscar retorno al spawn
SPAWN_DIST = 0.85      # m — considerado "en el spawn"
SPAWN_YAW  = 0.50      # rad — orientación similar al inicio

# Umbral de paralelismo: |df - db| > THRESH → corregir
PAR_THRESH = 0.10

HZ         = 10        # Hz del bucle de control
MAP_HZ     = 3.0       # Hz de publicación de mapa

# Resolución del mapa propio
MAP_RES    = 0.08      # m/celda
MAP_INIT   = 40.0      # m — tamaño inicial del grid

# ══════════════════════════════════════════════════════════════════════════════
# GEOMETRÍA LIDAR
# Ray 0=frente, 90=izq, 180=atrás, 270=dcha  (antihorario)
# ══════════════════════════════════════════════════════════════════════════════
if SIDE == "right":
    R_PERP   = 270   # perpendicular a la pared derecha
    R_DIAG_F = 315   # diagonal adelante-derecha (45° adelante del perp)
    R_DIAG_B = 225   # diagonal atrás-derecha   (45° atrás del perp)
    CMD_AWAY = 'q'   # girar lejos de pared = izquierda
    CMD_WALL = 'e'   # girar hacia pared    = derecha
else:
    R_PERP   = 90
    R_DIAG_F = 45
    R_DIAG_B = 135
    CMD_AWAY = 'e'
    CMD_WALL = 'q'

# ══════════════════════════════════════════════════════════════════════════════
# FUNCIONES LIDAR
# ══════════════════════════════════════════════════════════════════════════════

def _get(ranges, deg):
    """Rayo más cercano al ángulo deg (grados)."""
    n   = len(ranges)
    idx = int(round(deg % 360 * n / 360.0)) % n
    v   = ranges[idx]
    if v != v or v <= 0.01 or v == float('inf'):
        return 11.0
    return min(float(v), 11.0)

def _min(ranges, center, half):
    """Mínimo del sector [center-half, center+half] grados."""
    n    = len(ranges)
    step = 360.0 / n
    mn   = 11.0
    a    = center - half
    while a <= center + half:
        mn = min(mn, _get(ranges, a))
        a += step
    return mn

def _mean_close(ranges, center, half, max_d=4.0):
    """Media de rayos < max_d en el sector."""
    n    = len(ranges)
    step = 360.0 / n
    vals = []
    a    = center - half
    while a <= center + half:
        r = _get(ranges, a)
        if r < max_d:
            vals.append(r)
        a += step
    return float(np.mean(vals)) if vals else 11.0

def _adiff(a, b):
    d = a - b
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

# ══════════════════════════════════════════════════════════════════════════════
# SLAM — grid dinámico para publicar /map en RViz
# ══════════════════════════════════════════════════════════════════════════════

class MiniSlam:
    """SLAM mínimo: log-odds grid que crece automáticamente."""
    L_FREE, L_OCC, L_PRIOR = -0.4, 0.7, 0.0
    L_MIN,  L_MAX           = -3.5, 3.5
    FREE_P, OCC_P           = 0.35, 0.65

    def __init__(self, size_m, res):
        self.res = res
        n = int(size_m / res)
        self._ox = -size_m/2; self._oy = -size_m/2
        self._w = n; self._h = n
        self._lo = np.full((n, n), self.L_PRIOR, dtype=np.float32)

    @property
    def ox(self): return self._ox
    @property
    def oy(self): return self._oy
    @property
    def w(self):  return self._w
    @property
    def h(self):  return self._h

    def _expand(self, wx, wy, margin=0.0):
        mc = int(math.ceil(margin / self.res))
        cx = (wx - self._ox) / self.res
        cy = (wy - self._oy) / self.res
        pl = max(0, mc - int(cx))
        pr = max(0, int(math.ceil(cx)) + mc - (self._w - 1))
        pd = max(0, mc - int(cy))
        pu = max(0, int(math.ceil(cy)) + mc - (self._h - 1))
        if pl == pr == pd == pu == 0:
            return
        nw, nh = self._w + pl + pr, self._h + pd + pu
        nlo = np.full((nh, nw), self.L_PRIOR, dtype=np.float32)
        nlo[pd:pd+self._h, pl:pl+self._w] = self._lo
        self._lo = nlo; self._w = nw; self._h = nh
        self._ox -= pl * self.res; self._oy -= pd * self.res

    def _bres(self, x0, y0, x1, y1):
        pts = []; dx, dy = abs(x1-x0), abs(y1-y0)
        sx = 1 if x1>x0 else -1; sy = 1 if y1>y0 else -1; err = dx-dy
        while not (x0==x1 and y0==y1):
            pts.append((x0, y0)); e2 = 2*err
            if e2 > -dy: err -= dy; x0 += sx
            if e2 <  dx: err += dx; y0 += sy
        return pts

    def integrate(self, rx, ry, ryaw, ranges):
        self._expand(rx, ry, 8.0)
        n    = len(ranges)
        step = 2*math.pi / n
        for i, r in enumerate(ranges):
            ang  = ryaw + i * step
            reff = min(r if (r==r and r!=float('inf')) else 11.0, 11.0)
            ex   = rx + reff * math.cos(ang)
            ey   = ry + reff * math.sin(ang)
            self._expand(ex, ey, 1.0)
            rcx = int((rx - self._ox) / self.res)
            rcy = int((ry - self._oy) / self.res)
            ecx = int((ex - self._ox) / self.res)
            ecy = int((ey - self._oy) / self.res)
            for bx, by in self._bres(rcx, rcy, ecx, ecy):
                if 0 <= bx < self._w and 0 <= by < self._h:
                    self._lo[by, bx] = max(self.L_MIN,
                                           self._lo[by, bx] + self.L_FREE)
            if 0.15 < r < 11.0:
                if 0 <= ecx < self._w and 0 <= ecy < self._h:
                    self._lo[ecy, ecx] = min(self.L_MAX,
                                             self._lo[ecy, ecx] + self.L_OCC)

    def snapshot(self):
        return self._lo.copy(), self._ox, self._oy, self._w, self._h

# ══════════════════════════════════════════════════════════════════════════════
# CÁMARA — anti-colisión secundaria
# ══════════════════════════════════════════════════════════════════════════════

class Cam:
    def __init__(self):
        self._blocked = False; self._on = True
        if _CAM:
            threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            ctx = zmq.Context(); s = ctx.socket(zmq.SUB)
            s.setsockopt(zmq.CONFLATE, 1)
            s.setsockopt_string(zmq.SUBSCRIBE, "")
            s.connect(f"tcp://{TCP_HOST}:{CAM_PORT}")
            while self._on:
                try:
                    raw = s.recv(zmq.NOBLOCK)
                    arr = np.frombuffer(raw, dtype=np.uint8)
                    f   = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
                    if f is not None:
                        h, w = f.shape
                        roi  = f[h//2:, w//3:2*w//3]
                        self._blocked = float(np.mean(roi < 28)) > 0.28
                except zmq.Again:
                    time.sleep(0.033)
        except Exception:
            pass

    @property
    def blocked(self): return self._blocked
    def stop(self): self._on = False

# ══════════════════════════════════════════════════════════════════════════════
# TCP
# ══════════════════════════════════════════════════════════════════════════════

def _tcp(cmd):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.20); s.connect((TCP_HOST, TCP_PORT))
            s.sendall(cmd.encode()); s.recv(32)
    except Exception:
        pass

# ══════════════════════════════════════════════════════════════════════════════
# NODO PRINCIPAL
# ══════════════════════════════════════════════════════════════════════════════

class WallFollower(Node):

    S_APPROACH  = "APPROACH"
    S_FOLLOW    = "FOLLOW"
    S_CORNER_IN = "CORNER_IN"
    S_DONE      = "DONE"

    def __init__(self):
        super().__init__("wall_follower")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry,  "/odom", self._odom, qos)
        self.create_subscription(LaserScan, "/scan", self._scan, qos)

        # Publicador de mapa
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=5,
                             durability=DurabilityPolicy.VOLATILE)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self._slam    = MiniSlam(MAP_INIT, MAP_RES)
        self._slam_lock = threading.Lock()

        # Sensores
        self._rx = self._ry = self._ryaw = 0.0
        self._have_odom = self._have_scan = False
        self._ranges    = [11.0] * 360

        # Spawn
        self._sx = self._sy = self._syaw = 0.0
        self._spawn_set = False

        # Odometría de distancia recorrida
        self._dist     = 0.0
        self._prev_pos = None
        self._moving   = False

        # FSM
        self._state   = self.S_APPROACH
        self._state_t = time.time()
        self._last    = "stop"

        # Cámara
        self._cam = Cam()

        # Stall
        self._stall_pos = (0.0, 0.0)
        self._stall_t   = time.time()
        self._rec       = []
        self._rec_i     = 0
        self._rec_tend  = 0.0

        self.create_timer(1.0 / HZ,    self._loop)
        self.create_timer(1.0 / MAP_HZ, self._pub_map)

        lado = "DERECHA" if SIDE == "right" else "IZQUIERDA"
        self.get_logger().info(
            f"\n╔══════════════════════════════════════╗"
            f"\n║   G1 WALL FOLLOWER  [{lado:8s}]   ║"
            f"\n╠══════════════════════════════════════╣"
            f"\n║  dist objetivo: {WALL_IDEAL}m                 ║"
            f"\n║  /map → RViz2 activo                 ║"
            f"\n╚══════════════════════════════════════╝"
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _odom(self, msg):
        self._rx  = msg.pose.pose.position.x
        self._ry  = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._ryaw = math.atan2(2*(q.w*q.z+q.x*q.y),
                                1-2*(q.y*q.y+q.z*q.z))
        self._have_odom = True
        if self._prev_pos:
            d = math.hypot(self._rx-self._prev_pos[0],
                           self._ry-self._prev_pos[1])
            if d < 0.5:
                self._dist += d
        self._prev_pos = (self._rx, self._ry)
        if not self._spawn_set and self._have_scan:
            self._sx, self._sy, self._syaw = self._rx, self._ry, self._ryaw
            self._spawn_set = True
            self.get_logger().info(
                f"Spawn guardado: ({self._rx:.2f},{self._ry:.2f})")

    def _scan(self, msg):
        self._ranges = [
            v if (v==v and v!=float('inf') and v>0.01) else 11.0
            for v in msg.ranges]
        self._have_scan = True
        with self._slam_lock:
            self._slam.integrate(self._rx, self._ry, self._ryaw, self._ranges)

    # ── publicar mapa ─────────────────────────────────────────────────────────

    def _pub_map(self):
        if not (self._have_odom and self._have_scan):
            return
        with self._slam_lock:
            lo, ox, oy, w, h = self._slam.snapshot()
        prob = 1.0 / (1.0 + np.exp(-lo))
        grid = np.full(lo.shape, -1, dtype=np.int8)
        grid[prob < 0.35] = 0
        grid[prob > 0.65] = 100
        msg = OccupancyGrid()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.header.frame_id    = "odom"
        msg.info.resolution    = MAP_RES
        msg.info.width         = w
        msg.info.height        = h
        msg.info.origin.position.x    = ox
        msg.info.origin.position.y    = oy
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self._map_pub.publish(msg)

    # ── bucle principal ────────────────────────────────────────────────────────

    def _loop(self):
        if not (self._have_odom and self._have_scan):
            return

        # Emergencia
        if self._state != self.S_DONE:
            e = self._emerg()
            if e:
                self._cmd(e); return

        # Recuperación
        if self._rec:
            self._do_rec(); return

        # Stall
        if self._state in (self.S_APPROACH, self.S_FOLLOW, self.S_CORNER_IN):
            moved = math.hypot(self._rx-self._stall_pos[0],
                               self._ry-self._stall_pos[1])
            if moved > 0.07:
                self._stall_pos = (self._rx, self._ry)
                self._stall_t   = time.time()
            elif time.time() - self._stall_t > 3.5:
                self._start_rec(); return

        # FSM
        if   self._state == self.S_APPROACH:  self._approach()
        elif self._state == self.S_FOLLOW:     self._follow()
        elif self._state == self.S_CORNER_IN:  self._corner_in()
        elif self._state == self.S_DONE:       self._cmd("stop")

    # ── APPROACH ──────────────────────────────────────────────────────────────

    def _approach(self):
        """
        Avanza RECTO hacia la pared más cercana.
        No gira para orientarse — eso lo hace el step inicial mínimo.
        Cuando la pared lateral está a WALL_IDEAL → FOLLOW.
        """
        side  = _mean_close(self._ranges, R_PERP, 25)
        front = _min(self._ranges, 0, 35)

        # Ya en posición
        if side <= WALL_IDEAL + 0.08:
            self.get_logger().info(f"Pared a {side:.2f}m → FOLLOW")
            self._go(self.S_FOLLOW)
            return

        # La pared está en el lado correcto y el frente está libre → avanzar
        if front > FRONT_TURN:
            self._cmd('w')
            self._moving = True
            return

        # Obstáculo al frente durante approach → pequeño giro
        self._cmd(CMD_AWAY)

    # ── FOLLOW ────────────────────────────────────────────────────────────────

    def _follow(self):
        """
        Sigue la pared lateral.

        Usa df (rayo diagonal adelante) y db (rayo diagonal atrás):
          df < db  → robot apunta hacia la pared → girar LEJOS
          df > db  → robot apunta lejos de la pared → girar HACIA
          df ≈ db  → paralelo → avanzar

        Además controla la distancia absoluta (side).
        """
        side  = _mean_close(self._ranges, R_PERP, 25)
        df    = _get(self._ranges, R_DIAG_F)
        db    = _get(self._ranges, R_DIAG_B)
        front = _min(self._ranges, 0, 35)

        # Retorno al spawn
        if self._moving and self._dist >= MIN_TRAVEL:
            if self._near_spawn():
                self.get_logger().info(
                    f"VUELTA COMPLETA  dist={self._dist:.1f}m")
                self._go(self.S_DONE)
                self._cmd("stop")
                return

        # 1. Esquina interior
        if front < FRONT_TURN:
            self.get_logger().info(f"Esquina interior front={front:.2f}m")
            self._go(self.S_CORNER_IN)
            return

        # 2. Demasiado cerca
        if side < WALL_CLOSE:
            self._cmd(CMD_AWAY)
            return

        # 3. Esquina exterior: df se dispara, la pared desaparece al frente
        if df > WALL_FAR * 2.0 and side < WALL_FAR:
            self._cmd(CMD_WALL)
            return

        # 4. Demasiado lejos
        if side > WALL_FAR:
            self._cmd(CMD_WALL)
            return

        # 5. Corrección de paralelismo
        diff = df - db   # >0 apunta lejos, <0 apunta hacia pared
        if diff < -PAR_THRESH:
            self._cmd(CMD_AWAY)
        elif diff > PAR_THRESH:
            self._cmd(CMD_WALL)
        else:
            self._cmd('w')
            self._moving = True

    # ── CORNER_IN ─────────────────────────────────────────────────────────────

    def _corner_in(self):
        front = _min(self._ranges, 0, 35)
        self._cmd(CMD_AWAY)
        if front > FRONT_TURN * 1.4:
            self.get_logger().info("Esquina resuelta → FOLLOW")
            self._go(self.S_FOLLOW)

    # ── EMERGENCIA ────────────────────────────────────────────────────────────

    def _emerg(self):
        front = _min(self._ranges, 0, 35)
        if front < FRONT_STOP or (self._cam.blocked and front < FRONT_STOP*1.3):
            left  = _min(self._ranges, 90,  50)
            right = _min(self._ranges, 270, 50)
            return 'q' if left >= right else 'e'
        return None

    # ── SPAWN ─────────────────────────────────────────────────────────────────

    def _near_spawn(self):
        d  = math.hypot(self._rx-self._sx, self._ry-self._sy)
        dy = abs(_adiff(self._ryaw, self._syaw))
        return d < SPAWN_DIST and dy < SPAWN_YAW

    # ── STALL ─────────────────────────────────────────────────────────────────

    def _start_rec(self):
        self.get_logger().warn("Stall → recuperando")
        ls = _min(self._ranges, 90,  50)
        rs = _min(self._ranges, 270, 50)
        rot = 'q' if ls >= rs else 'e'
        self._rec    = [('s', 0.7), (rot, 1.2), ('w', 0.4)]
        self._rec_i  = 0
        self._rec_tend = time.time() + 0.7
        self._stall_t  = time.time()
        self._stall_pos = (self._rx, self._ry)

    def _do_rec(self):
        if self._rec_i >= len(self._rec):
            self._rec = []; self._go(self.S_FOLLOW); return
        cmd, _ = self._rec[self._rec_i]
        self._cmd(cmd)
        if time.time() >= self._rec_tend:
            self._rec_i += 1
            if self._rec_i < len(self._rec):
                self._rec_tend = time.time() + self._rec[self._rec_i][1]

    # ── UTIL ──────────────────────────────────────────────────────────────────

    def _go(self, s):
        if s != self._state:
            self.get_logger().info(
                f"  {self._state} → {s}  "
                f"side={_mean_close(self._ranges,R_PERP,25):.2f}m  "
                f"dist={self._dist:.1f}m")
            self._state = s; self._state_t = time.time()

    def _cmd(self, c):
        self._last = c
        threading.Thread(target=_tcp, args=(c,), daemon=True).start()

    def destroy_node(self):
        _tcp("stop"); self._cam.stop()
        super().destroy_node()

# ══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
