import time
import math
import numpy as np
import onnxruntime as ort

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
import subprocess
import os
import signal

class HolosomaLocomotion:
    def __init__(self, model_path="fastsac_g1_29dof.onnx"):
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

    def get_target_positions(self, state, cmd):
        cmd_mag = math.sqrt(cmd['vx']**2 + cmd['vy']**2 + cmd['yaw']**2)
        if cmd_mag < 0.01:
            self.phase = np.array([math.pi, math.pi], dtype=np.float32)
        else:
            self.phase = (self.phase + self.phase_dt) % (2 * math.pi)

        obs = np.zeros(100, dtype=np.float32)
        obs[0:29] = self.last_action                                  
        obs[29:32] = np.array(state['gyro']) * 0.25                   
        obs[32] = cmd['yaw']                                          
        obs[33:35] = [cmd['vx'], cmd['vy']]                           
        obs[35:37] = np.cos(self.phase)                               
        obs[37:66] = (np.array(state['joint_pos']) - self.default_angles) 
        obs[66:95] = np.array(state['joint_vel']) * 0.05              
        obs[95:98] = np.array(state['gravity'])                       
        obs[98:100] = np.sin(self.phase)                              
        
        action = self.session.run(None, {self.input_name: np.expand_dims(obs, axis=0)})[0].squeeze() 
        self.last_action = action
        return self.default_angles + (action * 0.25)

low_state = None

def state_callback(msg: LowState_):
    global low_state
    low_state = msg

def quaternion_to_gravity(q):
    w, x, y, z = q[0], q[1], q[2], q[3]

    gx = -2 * (x*z - w*y)
    gy = -2 * (y*z + w*x)
    gz = -(1 - 2 * (x*x + y*y))
    return [gx, gy, gz]

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sim_path = os.path.join(script_dir, "simulator")
    
    print(f"[INFO] Lanzando el simulador desde: {sim_path}")
    
    sim_proc = subprocess.Popen(
        ["python3", "unitree_mujoco.py"],
        cwd=sim_path
    )

    time.sleep(0.1)

    ChannelFactoryInitialize(1, "lo") 
    
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state_callback, 10)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    controller = HolosomaLocomotion(model_path="fastsac_g1_29dof.onnx")
    cmd_msg = unitree_hg_msg_dds__LowCmd_()

    kp = [40.18, 99.10, 40.18, 99.10, 28.50, 28.50]*2 + [40.18, 28.50, 28.50] + [14.25, 14.25, 14.25, 14.25, 16.78, 16.78, 16.78]*2
    kd = [2.56, 6.31, 2.56, 6.31, 1.81, 1.81]*2 + [2.56, 1.81, 1.81] + [0.91, 0.91, 0.91, 0.91, 1.07, 1.07, 1.07]*2

    print("[INFO] Esperando datos del simulador...")
    while low_state is None:
        time.sleep(0.1)
    
    print("[INFO] ¡Conectado! Controlando al robot...")
    comandos = {'vx': 0.0, 'vy': 0.0, 'yaw': 0.0}

    try:
        while True:
            t_start = time.time()
            estado = {
                'gyro': low_state.imu_state.gyroscope,
                'gravity': quaternion_to_gravity(low_state.imu_state.quaternion),
                'joint_pos': [low_state.motor_state[i].q for i in range(29)],
                'joint_vel': [low_state.motor_state[i].dq for i in range(29)]
            }
            targets = controller.get_target_positions(estado, comandos)
            
            for i in range(29):
                cmd_msg.motor_cmd[i].q = targets[i]
                cmd_msg.motor_cmd[i].dq = 0.0
                cmd_msg.motor_cmd[i].kp = kp[i]
                cmd_msg.motor_cmd[i].kd = kd[i]
                cmd_msg.motor_cmd[i].tau = 0.0
            
            pub.Write(cmd_msg)
            time.sleep(max(0.0, (1.0 / 50.0) - (time.time() - t_start)))
            
    except KeyboardInterrupt:
        print("\n[INFO] Detenido.")
