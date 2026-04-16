import struct
import time
import threading
from threading import Thread

import cv2
import mujoco
import mujoco.viewer
import numpy as np
import zmq

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

# === DEBUG: verificar qué archivo cargó MuJoCo y dónde está el lidar ===
import os as _os
_abs_scene = _os.path.abspath(config.ROBOT_SCENE)
print(f"\n{'='*60}")
print(f"[DEBUG] CWD:          {_os.getcwd()}")
print(f"[DEBUG] ROBOT_SCENE:  {config.ROBOT_SCENE}")
print(f"[DEBUG] Ruta absoluta: {_abs_scene}")
print(f"[DEBUG] ¿Existe?:     {_os.path.exists(_abs_scene)}")
_lid_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "lidar_site")
if _lid_id >= 0:
    _lid_pos = mj_model.site_pos[_lid_id]
    print(f"[DEBUG] lidar_site pos en modelo cargado: {_lid_pos}")
else:
    print(f"[DEBUG] lidar_site NO ENCONTRADO en el modelo")
print(f"{'='*60}\n")
ZMQ_LIDAR_MAGIC = 0xDEAD1337
RGB_WIDTH = 640
RGB_HEIGHT = 480
CAMERA_NAME = "realsense"
PELVIS_BODY_NAME = "pelvis"

# =====================================================================
# CONFIGURACIÓN DEL LIDAR 360°
# =====================================================================
LIDAR_SITE_NAME  = "lidar_site"       # Site en el XML del robot
LIDAR_PUBLISH_HZ = 10.0               # Frecuencia de publicación (Hz)
LIDAR_MAX_RANGE  = 12.0               # Alcance máximo (metros)

# Resolución angular
LIDAR_N_AZIMUTH    = 360              # Rayos horizontales (1° por rayo)
LIDAR_N_ELEVATION  = 1                # Canales verticales (1 = LiDAR 2D plano)
LIDAR_ELEV_MIN_DEG = 0.0              # Elevación mínima (grados)
LIDAR_ELEV_MAX_DEG = 0.0              # Elevación máxima (grados)

# Grupos de geometría que detecta el LiDAR (1 = detectar, 0 = ignorar)
# Índice 0..5 → group 0..5 en MuJoCo
# Tus paredes y suelo están en group 3
LIDAR_GEOMGROUP = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)

# Body del robot a excluir de los rayos (evita auto-detección)
LIDAR_EXCLUDE_BODY = PELVIS_BODY_NAME
# =====================================================================


if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


# =====================================================================
# Utilidades de pose
# =====================================================================
def _mat9_to_quat_wxyz(mat9: np.ndarray) -> np.ndarray:
    """Convierte una matriz 3x3 o 9 elementos a quat (w,x,y,z)."""
    mat9 = np.asarray(mat9, dtype=np.float64).reshape(-1)
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, mat9)
    return quat


def _pose_from_body(body_name: str) -> np.ndarray:
    """Devuelve pose [x,y,z,qw,qx,qy,qz] del body en world."""
    body_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise RuntimeError(f"No existe el body '{body_name}' en el modelo")
    pos = np.asarray(mj_data.xpos[body_id], dtype=np.float64)
    quat = _mat9_to_quat_wxyz(mj_data.xmat[body_id])
    return np.concatenate([pos, quat])


def _pose_from_site(site_name: str) -> np.ndarray:
    """Devuelve pose [x,y,z,qw,qx,qy,qz] del site en world."""
    site_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if site_id < 0:
        raise RuntimeError(f"No existe el site '{site_name}' en el modelo")
    pos = np.asarray(mj_data.site_xpos[site_id], dtype=np.float64)
    quat = _mat9_to_quat_wxyz(mj_data.site_xmat[site_id])
    return np.concatenate([pos, quat])


# =====================================================================
# LiDAR 360° por ray-casting
# =====================================================================
def _build_ray_directions(n_azimuth: int, n_elevation: int,
                          elev_min_deg: float, elev_max_deg: float) -> np.ndarray:
    """
    Pre-calcula las direcciones de los rayos en el frame local del LiDAR.
    Retorna array (N, 3) con direcciones unitarias.

    Convención del frame local del site en MuJoCo:
      +X = frente, +Y = izquierda, +Z = arriba
    El azimut gira alrededor de Z, la elevación sube/baja desde el plano XY.
    """
    azimuths = np.linspace(0, 2 * np.pi, n_azimuth, endpoint=False)

    if n_elevation <= 1:
        elevations = np.array([0.0])
    else:
        elevations = np.deg2rad(
            np.linspace(elev_min_deg, elev_max_deg, n_elevation))

    dirs = []
    for elev in elevations:
        cos_e = np.cos(elev)
        sin_e = np.sin(elev)
        for az in azimuths:
            dx = cos_e * np.cos(az)
            dy = cos_e * np.sin(az)
            dz = sin_e
            dirs.append([dx, dy, dz])

    return np.array(dirs, dtype=np.float64)


# Pre-calcular direcciones (no cambian entre frames)
_ray_dirs_local = _build_ray_directions(
    LIDAR_N_AZIMUTH, LIDAR_N_ELEVATION,
    LIDAR_ELEV_MIN_DEG, LIDAR_ELEV_MAX_DEG)

# Resolver IDs una sola vez
_lidar_site_id = mujoco.mj_name2id(
    mj_model, mujoco.mjtObj.mjOBJ_SITE, LIDAR_SITE_NAME)
if _lidar_site_id < 0:
    raise RuntimeError(f"No existe el site '{LIDAR_SITE_NAME}' en el modelo. "
                       f"Asegúrate de que está definido en g1_29dof.xml")

_exclude_body_id = mujoco.mj_name2id(
    mj_model, mujoco.mjtObj.mjOBJ_BODY, LIDAR_EXCLUDE_BODY)


def _raycast_360(model, data) -> tuple:
    """
    Ejecuta el ray-casting 360° desde el lidar_site.
    Retorna (pts_local, lidar_pose, pelvis_pose).
      pts_local: (N, 3) float32 — puntos en frame lidar_link
    """
    # Posición y rotación del site en world
    site_pos = data.site_xpos[_lidar_site_id].copy()
    site_mat = data.site_xmat[_lidar_site_id].reshape(3, 3)

    # Rotar las direcciones locales al frame world
    dirs_world = (_ray_dirs_local @ site_mat.T)

    pts_local = []
    geomid_out = np.array([-1], dtype=np.int32)

    for i in range(len(dirs_world)):
        dist = mujoco.mj_ray(
            model, data,
            site_pos,                   # origen del rayo
            dirs_world[i],              # dirección en world
            LIDAR_GEOMGROUP,            # grupos a detectar
            1,                          # flg_static = 1
            _exclude_body_id,           # excluir body del robot
            geomid_out,                 # output: id del geom impactado
        )

        if 0 < dist < LIDAR_MAX_RANGE:
            # Punto de impacto en world
            hit_world = site_pos + dist * dirs_world[i]
            # Convertir a frame local del LiDAR
            hit_local = site_mat.T @ (hit_world - site_pos)
            pts_local.append(hit_local)

    if pts_local:
        pts = np.array(pts_local, dtype=np.float32)
    else:
        pts = np.empty((0, 3), dtype=np.float32)

    lidar_pose = _pose_from_site(LIDAR_SITE_NAME)
    pelvis_pose = _pose_from_body(PELVIS_BODY_NAME)

    return pts, lidar_pose, pelvis_pose


# =====================================================================
# Protocolo ZMQ (mismo formato que el bridge ROS2 espera)
# =====================================================================
def _pack_lidar_message(pelvis_pose: np.ndarray, lidar_pose: np.ndarray,
                        pts: np.ndarray) -> bytes:
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    pelvis_pose = np.ascontiguousarray(pelvis_pose, dtype=np.float64)
    lidar_pose = np.ascontiguousarray(lidar_pose, dtype=np.float64)

    return b"".join([
        struct.pack("=II", ZMQ_LIDAR_MAGIC, int(len(pts))),
        pelvis_pose.tobytes(),
        lidar_pose.tobytes(),
        struct.pack("=d", time.time()),
        pts.tobytes(),
    ])


# =====================================================================
# Threads
# =====================================================================
def ResetServerThread():
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('127.0.0.1', 6005))
    sock.settimeout(0.5)
    print("[INFO] Servidor de Teletransporte activo en puerto 6005")

    while viewer.is_running():
        try:
            data, _ = sock.recvfrom(1024)
            if data == b"reset":
                print("\n[INFO] Teletransportando G1...")
                with locker:
                    mj_data.qpos[2] = 0.793
                    mj_data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
                    mj_data.qpos[7:] = 0.0
                    mj_data.qvel[:] = 0.0
                    mujoco.mj_forward(mj_model, mj_data)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[ERROR] Fallo en teletransporte: {e}")


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


def RGBServerThread():
    """Servidor ZMQ para la imagen RGB de la cámara (puerto 5555)."""
    context = zmq.Context()
    socket_rgb = context.socket(zmq.PUB)
    socket_rgb.bind("tcp://*:5555")

    renderer_rgb = mujoco.Renderer(mj_model, height=RGB_HEIGHT, width=RGB_WIDTH)

    print("[INFO] Servidor RGB en puerto 5555")

    while viewer.is_running():
        try:
            with locker:
                renderer_rgb.update_scene(mj_data, camera=CAMERA_NAME)
                pixels_rgb = renderer_rgb.render()

            bgr_frame = cv2.cvtColor(pixels_rgb, cv2.COLOR_RGB2BGR)
            _, rgb_buffer = cv2.imencode('.jpg', bgr_frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, 80])
            socket_rgb.send(rgb_buffer.tobytes())
        except Exception as e:
            print(f"[WARN] RGB thread: {e}")

        time.sleep(1.0 / 30.0)


def LidarThread():
    """
    LiDAR 360° por ray-casting.
    Publica nube de puntos + poses vía ZMQ (puerto 5556).
    """
    context = zmq.Context()
    socket_lidar = context.socket(zmq.PUB)
    socket_lidar.bind("tcp://*:5556")

    publish_period = 1.0 / LIDAR_PUBLISH_HZ
    n_rays = len(_ray_dirs_local)

    print(f"[INFO] LiDAR 360° activo en puerto 5556")
    print(f"       Rayos: {n_rays} ({LIDAR_N_AZIMUTH} az x {LIDAR_N_ELEVATION} elev)")
    print(f"       Rango máximo: {LIDAR_MAX_RANGE} m")
    print(f"       Frecuencia: {LIDAR_PUBLISH_HZ} Hz")
    print(f"       Grupos detectados: {list(LIDAR_GEOMGROUP)}")

    while viewer.is_running():
        t0 = time.perf_counter()

        try:
            with locker:
                pts, lidar_pose, pelvis_pose = _raycast_360(mj_model, mj_data)

            payload = _pack_lidar_message(pelvis_pose, lidar_pose, pts)
            socket_lidar.send(payload)

        except Exception as e:
            print(f"[WARN] LiDAR thread: {e}")

        elapsed = time.perf_counter() - t0
        sleep_time = publish_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    rgb_thread = Thread(target=RGBServerThread)
    rgb_thread.daemon = True

    lidar_thread = Thread(target=LidarThread)
    lidar_thread.daemon = True

    reset_thread = Thread(target=ResetServerThread)
    reset_thread.daemon = True

    viewer_thread.start()
    sim_thread.start()
    rgb_thread.start()
    lidar_thread.start()
    reset_thread.start()

    print("\n[INFO] Todos los hilos arrancados.")
    print("[INFO] Pulsa tecla '3' en el visor MuJoCo para ver/ocultar muros (group 3)")
