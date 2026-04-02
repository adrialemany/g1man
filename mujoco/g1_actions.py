import time
import threading
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

L_PITCH, L_ROLL, L_YAW, L_ELBOW = 15, 16, 17, 18
R_PITCH, R_ROLL, R_YAW, R_ELBOW = 22, 23, 24, 25

REST_POS = {
    R_PITCH: 0.3, R_ROLL: -0.2, R_YAW: 0.0, R_ELBOW: 0.5,
    L_PITCH: 0.3, L_ROLL: 0.2, L_YAW: 0.0, L_ELBOW: 0.5
}

cmd = unitree_hg_msg_dds__LowCmd_()

def init_robot():
    """Inicialización con los índices corregidos"""
    cmd.mode_pr = 0
    cmd.mode_machine = 2
    for i in range(35):
        cmd.motor_cmd[i].mode = 1
        cmd.motor_cmd[i].kp = 40.0
        cmd.motor_cmd[i].kd = 1.0
        cmd.motor_cmd[i].q = 0.0
    
    for idx, pos in REST_POS.items():
        cmd.motor_cmd[idx].q = pos

def move_to(target_positions, duration):
    steps = int(duration / 0.02)
    start_positions = {i: cmd.motor_cmd[i].q for i in target_positions.keys()}
    for s in range(1, steps + 1):
        ratio = s / steps
        for i, target_q in target_positions.items():
            cmd.motor_cmd[i].q = start_positions[i] + (target_q - start_positions[i]) * ratio
        time.sleep(0.02)

def udp_publisher_loop(pub):
    while True:
        pub.Write(cmd)
        time.sleep(0.02)

def action_0_wave():
    print(">> Saludando (Wave) con brazo derecho...")
    move_to({
        R_PITCH: -3.5, 
        R_ROLL: -1.3, 
        R_ELBOW: 0.0, 
        R_YAW: 2.0
    }, 2.0)

    for _ in range(3):
        move_to({R_ELBOW: 0.3}, 0.25)
        move_to({R_ELBOW: -0.3}, 0.25)

    move_to(REST_POS, 2.0)

def action_1_handshake():
    print(">> Dando la mano con brazo derecho...")

    move_to({
        R_PITCH: -0.5, 
        R_ROLL: -0.3, 
        R_ELBOW: 0.5, 
        R_YAW: 0.5
    }, 1.0)
    time.sleep(1.0)

    for _ in range(3):
        move_to({R_ELBOW: 0.3}, 0.2)
        move_to({R_ELBOW: 0.7}, 0.2)
    time.sleep(2.0)
    move_to(REST_POS, 2.0)

def main():
    ChannelFactoryInitialize(1, "lo")
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    init_robot()
    t = threading.Thread(target=udp_publisher_loop, args=(pub,), daemon=True)
    t.start()

    print("\n--- G1 CONTROL (29 DOF CORREGIDO) ---")
    print("Brazos en 15-18 (L) y 22-25 (R)")
    
    while True:
        op = input("\n[0: Saludar, 1: Mano, q: Salir]: ")
        if op == '0': action_0_wave()
        elif op == '1': action_1_handshake()
        elif op == 'q': break

if __name__ == '__main__':
    main()
