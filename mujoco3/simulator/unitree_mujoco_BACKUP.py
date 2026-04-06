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

ZMQ_LIDAR_MAGIC = 0xDEAD1337
RGB_WIDTH = 640
RGB_HEIGHT = 480
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
DEPTH_MAX_RANGE_M = 8.0
DEPTH_MIN_RANGE_M = 0.10
POINT_STRIDE = 4          # 640x480 -> ~19k puntos
POINT_PUBLISH_HZ = 10.0
CAMERA_NAME = "realsense"
PELVIS_BODY_NAME = "pelvis"


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


def _mat9_to_quat_wxyz(mat9: np.ndarray) -> np.ndarray:
    """Convierte una matriz 3x3 o 9 elementos a quat MuJoCo/ROS en orden (w,x,y,z)."""
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


def _pose_from_camera(camera_name: str) -> np.ndarray:
    """Devuelve pose [x,y,z,qw,qx,qy,qz] de la cámara en world."""
    cam_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        raise RuntimeError(f"No existe la cámara '{camera_name}' en el modelo")

    pos = np.asarray(mj_data.cam_xpos[cam_id], dtype=np.float64)
    quat = _mat9_to_quat_wxyz(mj_data.cam_xmat[cam_id])
    return np.concatenate([pos, quat])


def _depth_to_pointcloud(depth_map: np.ndarray, fovy_deg: float, stride: int = 4) -> np.ndarray:
    """
    Proyecta depth image (en metros) a nube XYZ en el frame óptico de la cámara.
    Se asume convención típica de cámara: +X derecha, +Y abajo, +Z hacia delante.
    """
    h, w = depth_map.shape[:2]
    fy = 0.5 * h / np.tan(np.deg2rad(fovy_deg) * 0.5)
    fx = fy
    cx = (w - 1) * 0.5
    cy = (h - 1) * 0.5

    uu, vv = np.meshgrid(
        np.arange(0, w, stride, dtype=np.float32),
        np.arange(0, h, stride, dtype=np.float32),
        indexing="xy",
    )

    z = depth_map[::stride, ::stride].astype(np.float32)
    mask = np.isfinite(z) & (z > DEPTH_MIN_RANGE_M) & (z < DEPTH_MAX_RANGE_M)
    if not np.any(mask):
        return np.empty((0, 3), dtype=np.float32)

    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy

    pts = np.stack((x, y, z), axis=-1)
    return np.ascontiguousarray(pts[mask], dtype=np.float32)


def _pack_lidar_message(pelvis_pose: np.ndarray, lidar_pose: np.ndarray, pts: np.ndarray) -> bytes:
    """
    Formato esperado por mujoco_ros2_lidar_bridge.py:
      [magic:uint32][n_pts:uint32]
      [pelvis_pose:7*float64]
      [lidar_pose:7*float64]
      [timestamp:float64]
      [points:n*3*float32]
    """
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
                print("\n[INFO] 💥 ¡Simulador recibe orden! Teletransportando a G1...")
                with locker:
                    # 1. Elevamos a la altura de la cadera (Mantenemos X e Y)
                    mj_data.qpos[2] = 0.793

                    # 2. Reseteamos la orientación (Quaternion: W, X, Y, Z) para ponerlo vertical
                    mj_data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

                    # 3. Ponemos TODOS los motores en su punto 0 (Totalmente rectos)
                    mj_data.qpos[7:] = 0.0

                    # 4. Eliminamos toda inercia de caída
                    mj_data.qvel[:] = 0.0

                    # 5. OBLIGATORIO: Avisar a MuJoCo de que hemos hackeado las físicas
                    mujoco.mj_forward(mj_model, mj_data)

        except socket.timeout:
            pass
        except Exception as e:
            print(f"[ERROR] Fallo en el teletransporte de MuJoCo: {e}")


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


def VisionServerThread():
    context = zmq.Context()

    # Puerto clásico para la imagen RGB
    socket_rgb = context.socket(zmq.PUB)
    socket_rgb.bind("tcp://*:5555")

    # Puerto para nube de puntos + poses (consumido por ROS2 bridge)
    socket_lidar = context.socket(zmq.PUB)
    socket_lidar.bind("tcp://*:5556")

    renderer_rgb = mujoco.Renderer(mj_model, height=RGB_HEIGHT, width=RGB_WIDTH)
    renderer_depth = mujoco.Renderer(mj_model, height=DEPTH_HEIGHT, width=DEPTH_WIDTH)
    renderer_depth.enable_depth_rendering()

    cam_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)
    if cam_id < 0:
        raise RuntimeError(f"No existe la cámara '{CAMERA_NAME}' en el modelo")
    fovy_deg = float(mj_model.cam_fovy[cam_id])

    print("[INFO] Servidor RGB en 5555 | Servidor PointCloud+Pose en 5556")

    publish_period = 1.0 / POINT_PUBLISH_HZ
    last_pc_pub = 0.0

    while viewer.is_running():
        try:
            with locker:
                renderer_rgb.update_scene(mj_data, camera=CAMERA_NAME)
                renderer_depth.update_scene(mj_data, camera=CAMERA_NAME)
                pixels_rgb = renderer_rgb.render()
                depth_map = renderer_depth.render()

                pelvis_pose = _pose_from_body(PELVIS_BODY_NAME)
                camera_pose = _pose_from_camera(CAMERA_NAME)

            bgr_frame = cv2.cvtColor(pixels_rgb, cv2.COLOR_RGB2BGR)
            _, rgb_buffer = cv2.imencode('.jpg', bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            socket_rgb.send(rgb_buffer.tobytes())

            now = time.perf_counter()
            if now - last_pc_pub >= publish_period:
                pts = _depth_to_pointcloud(depth_map, fovy_deg=fovy_deg, stride=POINT_STRIDE)
                payload = _pack_lidar_message(pelvis_pose, camera_pose, pts)
                socket_lidar.send(payload)
                last_pc_pub = now

        except Exception as e:
            print(f"[WARN] Vision/LiDAR thread: {e}")

        time.sleep(1.0 / 30.0)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)
    vision_thread = Thread(target=VisionServerThread)
    vision_thread.daemon = True

    reset_thread = Thread(target=ResetServerThread)
    reset_thread.daemon = True

    viewer_thread.start()
    sim_thread.start()
    vision_thread.start()
    reset_thread.start()
