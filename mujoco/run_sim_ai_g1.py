import os
import sys
import time
import math
import numpy as np
import onnxruntime as ort
import subprocess
import socket
import json
import threading
import atexit

# --- CONFIGURACIÓN ROS 2 ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# --- CONFIGURACIÓN CRÍTICA: Desactivar SharedMemory para evitar temblores y crashes ---
os.environ["CYCLONEDDS_URI"] = """<CycloneDDS>
    <Domain>
        <SharedMemory>
            <Enable>false</Enable>
        </SharedMemory>
    </Domain>
</CycloneDDS>"""

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

# =======================================================================
# CLASE PUENTE: ENVÍA LA REALIDAD DE MUJOCO A MOVEIT
# =======================================================================
class ROS2StateBridge(Node):
    def __init__(self):
        super().__init__('sim_state_bridge')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        # Nombres exactos del URDF de Unitree G1
        self.names = [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        ]

    def publish_state(self, q_list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [float(x) for x in q_list]
        self.pub.publish(msg)

# =======================================================================
# CLASE LOCOMOCIÓN: INFERENCIA DEL MODELO ONNX
# =======================================================================
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

# --- GLOBALES ---
low_state = None
external_arm_targets = {}
comandos = {'vx': 0.0, 'vy': 0.0, 'yaw': 0.0}
background_processes = []

def cleanup_processes():
    print("\n[INFO] Cerrando servicios en segundo plano...")
    for p in background_processes:
        try: p.terminate()
        except: pass
    try: rclpy.shutdown()
    except: pass

atexit.register(cleanup_processes)

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
            elif data == 'stop':
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

def launch_background_services(script_dir):
    print("[INFO] Levantando ecosistema MoveIt y ROS 2...")
    # Cargar Workspaces
    setup_cmd = "source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash && "
    
    # 1. Perception Bridge (Nube de puntos / Cámaras)
    p_bridge = subprocess.Popen(setup_cmd + "python3 simulator/perception_bridge.py", shell=True, executable='/bin/bash', cwd=script_dir)
    background_processes.append(p_bridge)
    
    # 2. Núcleo MoveIt (Headless)
    p_moveit = subprocess.Popen(setup_cmd + "ros2 launch g1_moveit_config demo.launch.py use_rviz:=false", shell=True, executable='/bin/bash', cwd=script_dir)
    background_processes.append(p_moveit)

    print("[INFO] Servicios ROS 2 lanzados.")
    time.sleep(0.1)

# =======================================================================
# EJECUCIÓN PRINCIPAL
# =======================================================================
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sim_path = os.path.join(script_dir, "simulator")
    model_path = os.path.join(script_dir, "fastsac_g1_29dof.onnx")
    
    # Iniciar ROS 2 para el puente de feedback
    rclpy.init()
    bridge_node = ROS2StateBridge()
    threading.Thread(target=lambda: rclpy.spin(bridge_node), daemon=True).start()

    print(f"[INFO] Lanzando el simulador MuJoCo...")
    sim_proc = subprocess.Popen(["python3", "unitree_mujoco.py"], cwd=sim_path)
    background_processes.append(sim_proc)
    time.sleep(1.0)

    # Lanzar servicios acoplados
    launch_background_services(script_dir)

    # Oyentes de sockets
    threading.Thread(target=locomotion_listener, daemon=True).start()
    threading.Thread(target=external_arm_listener, daemon=True).start()

    # Inicializar DDS de Unitree
    ChannelFactoryInitialize(1, "lo") 
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_callback, 10)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    controller = HolosomaLocomotion(model_path=model_path)
    
    # Ganancias PD
    kp = [40.18, 99.10, 40.18, 99.10, 28.50, 28.50]*2 + [40.18, 28.50, 28.50] + [14.25, 14.25, 14.25, 14.25, 16.78, 16.78, 16.78]*2
    kd = [2.56, 6.31, 2.56, 6.31, 1.81, 1.81]*2 + [2.56, 1.81, 1.81] + [0.91, 0.91, 0.91, 0.91, 1.07, 1.07, 1.07]*2

    print("[INFO] Esperando conexión con la simulación...")
    while low_state is None:
        time.sleep(0.1)
    
    print("[INFO] ¡Control activo!")

    try:
        while True:
            t_start = time.time()
            cmd_msg = unitree_hg_msg_dds__LowCmd_() 
            
            # Recolectar estado actual
            q_actual = [low_state.motor_state[i].q for i in range(29)]
            estado = {
                'gyro': low_state.imu_state.gyroscope,
                'gravity': quaternion_to_gravity(low_state.imu_state.quaternion),
                'joint_pos': q_actual,
                'joint_vel': [low_state.motor_state[i].dq for i in range(29)]
            }

            # --- FEEDBACK PARA MOVEIT (CRÍTICO) ---
            bridge_node.publish_state(q_actual)

            # Detección de caída
            if estado['gravity'][2] > -0.5:
                print("🚨 Caída detectada. Reseteando...")
                try:
                    sock_reset = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock_reset.sendto(b"reset", ("127.0.0.1", 6005))
                    sock_reset.close()
                except: pass
                comandos['vx'] = 0.0; comandos['vy'] = 0.0; comandos['yaw'] = 0.0
                time.sleep(0.1)
                controller.phase = np.array([0.0, math.pi], dtype=np.float32)
                continue
            
            # Cálculo de la IA + Mezcla con MoveIt
            targets = controller.get_target_positions(estado, comandos, external_arm_targets)
            
            for i in range(29):
                cmd_msg.motor_cmd[i].mode = 1
                cmd_msg.motor_cmd[i].dq = 0.0
                cmd_msg.motor_cmd[i].kp = kp[i]
                cmd_msg.motor_cmd[i].kd = kd[i]
                cmd_msg.motor_cmd[i].tau = 0.0
                
                # Prioridad absoluta a los comandos del brazo derecho de MoveIt
                if i in external_arm_targets:
                    cmd_msg.motor_cmd[i].q = external_arm_targets[i]
                else:
                    cmd_msg.motor_cmd[i].q = targets[i]
            
            pub.Write(cmd_msg)
            
            # Mantener 50Hz clavados
            time.sleep(max(0.0, (1.0 / 50.0) - (time.time() - t_start)))
            
    except KeyboardInterrupt:
        print("\n[INFO] Apagando...")
