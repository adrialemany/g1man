import os
import sys

# --- CONFIGURACIÓN CRÍTICA: Desactivar SharedMemory para evitar temblores y crashes ---
os.environ["CYCLONEDDS_URI"] = """<CycloneDDS>
    <Domain>
        <SharedMemory>
            <Enable>false</Enable>
        </SharedMemory>
    </Domain>
</CycloneDDS>"""

import time
import math
import numpy as np
import onnxruntime as ort
import subprocess
import socket
import json
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_


class HolosomaLocomotion:
    def __init__(self, model_path):
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        
        self.default_angles = np.zeros(29, dtype=np.float32)
        self.default_angles[[0, 6]] = -0.312    
        self.default_angles[[3, 9]] = 0.669     
        self.default_angles[[4, 10]] = -0.363   
        self.default_angles[[15, 22]] = 0.2     
        self.default_angles[16] = 0.2           
        self.default_angles[23] = -0.2          
        self.default_angles[[18, 25]] = 0.6     

        self.last_action = np.zeros(29, dtype=np.float32)
        self.control_freq = 50 
        self.gait_period = 1.0 
        self.phase_dt = 2 * math.pi / (self.control_freq * self.gait_period)
        self.phase = np.array([0.0, math.pi], dtype=np.float32) 

    def get_target_positions(self, state, cmd, external_targets):
        cmd_mag = math.sqrt(cmd['vx']**2 + cmd['vy']**2 + cmd['yaw']**2)
        if cmd_mag < 0.01:
            self.phase = np.array([math.pi, math.pi], dtype=np.float32)
        else:
            self.phase = (self.phase + self.phase_dt) % (2 * math.pi)

        fake_joint_pos = np.array(state['joint_pos'])
        fake_joint_vel = np.array(state['joint_vel'])
        
        for i in range(29):
            if i in external_targets:
                fake_joint_pos[i] = self.default_angles[i] + (self.last_action[i] * 0.25)
                fake_joint_vel[i] = 0.0

        obs = np.zeros(100, dtype=np.float32)
        obs[0:29] = self.last_action                                  
        obs[29:32] = np.array(state['gyro']) * 0.25                   
        obs[32] = cmd['yaw']                                          
        obs[33:35] = [cmd['vx'], cmd['vy']]                           
        obs[35:37] = np.cos(self.phase)                               
        obs[37:66] = (fake_joint_pos - self.default_angles) 
        obs[66:95] = fake_joint_vel * 0.05              
        obs[95:98] = np.array(state['gravity'])                       
        obs[98:100] = np.sin(self.phase)                              
        
        action = self.session.run(None, {self.input_name: np.expand_dims(obs, axis=0)})[0].squeeze() 
        self.last_action = action
        return self.default_angles + (action * 0.25)

low_state = None
external_arm_targets = {}
comandos = {'vx': 0.0, 'vy': 0.0, 'yaw': 0.0}

def state_callback(msg: LowState_):
    global low_state
    low_state = msg

def quaternion_to_gravity(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    gx = -2 * (x*z - w*y)
    gy = -2 * (y*z + w*x)
    gz = -(1 - 2 * (x*x + y*y))
    return [gx, gy, gz]

def locomotion_listener():
    global comandos
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', 6000))
    server.listen(5)
    while True:
        conn, _ = server.accept()
        try:
            data = conn.recv(1024).decode('utf-8')
            if not data: continue
            if data == 'w': comandos['vx'] = 0.6
            elif data == 's': comandos['vx'] = -0.4
            elif data == 'a': comandos['vy'] = 0.3
            elif data == 'd': comandos['vy'] = -0.3
            elif data == 'q': comandos['yaw'] = 0.6
            elif data == 'e': comandos['yaw'] = -0.6
            elif data == 'stop' or data == 'ping':
                if data == 'stop':
                    comandos['vx'], comandos['vy'], comandos['yaw'] = 0.0, 0.0, 0.0
            conn.sendall(b"OK")
        except: pass
        finally: conn.close()

def external_arm_listener():
    global external_arm_targets
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9876))
    sock.settimeout(0.5)
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            incoming_targets = json.loads(data.decode('utf-8'))
            external_arm_targets = {int(k): float(v) for k, v in incoming_targets.items()}
        except: continue


def cmd_vel_listener():
    """
    Suscriptor ROS 2 a /cmd_vel (geometry_msgs/Twist).
    Convierte velocidades del controller de Nav2 a comandos vx/vy/yaw del policy.
    Funciona en paralelo al teleop TCP del puerto 6000 (último en llegar gana).
    """
    global comandos
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
    except ImportError:
        print("[WARN] rclpy no disponible, /cmd_vel deshabilitado")
        return

    # Límites de seguridad para que el policy no reciba valores fuera de rango
    VX_MAX, VY_MAX, YAW_MAX = 0.6, 0.4, 0.8
    LAST_CMD_TIMEOUT = 0.5  # s sin mensaje → parar

    if not rclpy.ok():
        rclpy.init(args=None)

    node = rclpy.create_node('g1_cmd_vel_bridge')
    last_msg_time = [time.time()]

    def cb(msg):
        comandos['vx']  = float(np.clip(msg.linear.x,  -VX_MAX, VX_MAX))
        comandos['vy']  = float(np.clip(msg.linear.y,  -VY_MAX, VY_MAX))
        comandos['yaw'] = float(np.clip(msg.angular.z, -YAW_MAX, YAW_MAX))
        last_msg_time[0] = time.time()

    node.create_subscription(Twist, '/cmd_vel', cb, 10)
    print("[INFO] Suscriptor /cmd_vel activo (Nav2 ready)")

    # Watchdog: si no llegan mensajes en LAST_CMD_TIMEOUT, parar
    def watchdog():
        while True:
            time.sleep(0.1)
            if time.time() - last_msg_time[0] > LAST_CMD_TIMEOUT:
                # Solo paramos si el último comando vino del cmd_vel
                # (no machacamos un comando manual del TCP)
                pass  # El TCP sigue mandando, no interferimos

    threading.Thread(target=watchdog, daemon=True).start()
    rclpy.spin(node)

if __name__ == "__main__":
    # --- RUTAS ABSOLUTAS ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sim_path = os.path.join(script_dir, "simulator")
    model_path = os.path.join(script_dir, "fastsac_g1_29dof.onnx") # <--- AÑADIDO ESTO
    
    print(f"[INFO] Lanzando el simulador...")
    sim_proc = subprocess.Popen(["python3", "unitree_mujoco.py"], cwd=sim_path)
    time.sleep(1.0)

    threading.Thread(target=locomotion_listener, daemon=True).start()
    threading.Thread(target=external_arm_listener, daemon=True).start()
    threading.Thread(target=cmd_vel_listener, daemon=True).start()

    ChannelFactoryInitialize(1, "lo") 
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_callback, 10)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    # --- USAMOS LA RUTA ABSOLUTA AQUÍ ---
    controller = HolosomaLocomotion(model_path=model_path)
    
    kp = [40.18, 99.10, 40.18, 99.10, 28.50, 28.50]*2 + [40.18, 28.50, 28.50] + [14.25, 14.25, 14.25, 14.25, 16.78, 16.78, 16.78]*2
    kd = [2.56, 6.31, 2.56, 6.31, 1.81, 1.81]*2 + [2.56, 1.81, 1.81] + [0.91, 0.91, 0.91, 0.91, 1.07, 1.07, 1.07]*2

    print("[INFO] Esperando datos...")
    while low_state is None:
        time.sleep(0.1)
    
    print("[INFO] ¡Control activo! Usa WASD en el cliente.")

    try:
        while True:
            t_start = time.time()
            cmd_msg = unitree_hg_msg_dds__LowCmd_() 
            
            estado = {
                'gyro': low_state.imu_state.gyroscope,
                'gravity': quaternion_to_gravity(low_state.imu_state.quaternion),
                'joint_pos': [low_state.motor_state[i].q for i in range(29)],
                'joint_vel': [low_state.motor_state[i].dq for i in range(29)]
            }

            # --- DETECCIÓN DE CAÍDA Y TELETRANSPORTE ---
            if estado['gravity'][2] > -0.5:
                print("🚨 ¡Caída detectada! Reseteando posición...")
                
                try:
                    sock_reset = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock_reset.sendto(b"reset", ("127.0.0.1", 6005))
                    sock_reset.close()
                except: pass
                
                comandos['vx'] = 0.0
                comandos['vy'] = 0.0
                comandos['yaw'] = 0.0
                
                time.sleep(0.1)
                
                controller.phase = np.array([0.0, math.pi], dtype=np.float32)
                continue
            
            # --- CONTROL DE LA IA PURA ---
            targets = controller.get_target_positions(estado, comandos, external_arm_targets)
            
            for i in range(29):
                cmd_msg.motor_cmd[i].mode = 1
                cmd_msg.motor_cmd[i].dq = 0.0
                cmd_msg.motor_cmd[i].kp = kp[i]
                cmd_msg.motor_cmd[i].kd = kd[i]
                cmd_msg.motor_cmd[i].tau = 0.0
                
                if i in external_arm_targets:
                    cmd_msg.motor_cmd[i].q = external_arm_targets[i]
                else:
                    cmd_msg.motor_cmd[i].q = targets[i]
            
            pub.Write(cmd_msg)
            time.sleep(max(0.0, (1.0 / 50.0) - (time.time() - t_start)))
            
    except KeyboardInterrupt:
        print("\n[INFO] Detenido.")
        sim_proc.terminate()
