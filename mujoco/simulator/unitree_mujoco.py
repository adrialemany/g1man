import zmq
import cv2
import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)


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
            # Ahora, si falla algo al moverlo, ¡nos lo dirá la terminal!
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
    
    # Puerto clásico para la imagen (Para tu GUI y el cerebro)
    socket_rgb = context.socket(zmq.PUB)
    socket_rgb.bind("tcp://*:5555")
    
    # Nuevo puerto exclusivo para el Láser (Solo para el cerebro)
    socket_depth = context.socket(zmq.PUB)
    socket_depth.bind("tcp://*:5556")
    
    renderer_rgb = mujoco.Renderer(mj_model, height=480, width=640)
    renderer_depth = mujoco.Renderer(mj_model, height=480, width=640)
    renderer_depth.enable_depth_rendering()
    
    print("[INFO] Servidor RGB en 5555 | Servidor LÁSER en 5556")
    
    while viewer.is_running():
        try:
            with locker:
                renderer_rgb.update_scene(mj_data, camera="realsense")
                renderer_depth.update_scene(mj_data, camera="realsense")
            
            pixels_rgb = renderer_rgb.render()
            depth_map = renderer_depth.render()
            
            bgr_frame = cv2.cvtColor(pixels_rgb, cv2.COLOR_RGB2BGR)
            _, rgb_buffer = cv2.imencode('.jpg', bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Enviamos cada cosa por su tubería independiente
            socket_rgb.send(rgb_buffer.tobytes())
            socket_depth.send(depth_map.tobytes())
            
        except Exception as e:
            pass
        
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
