import time
import threading
import numpy as np
import pinocchio as pin
import json
import os
import sys
import socket
import termios
import tty
import select

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


class G1PerfectIK:

    def __init__(self):

        self.dt = 0.02
        self.state_received = False
        self.current_jpos = [0.0] * 29
        self.initialized = False

        self.last_key = None
        self.last_time = time.time()

        urdf_path = os.path.expanduser(
            "/home/david/g1_version2/src/g1man/src/g1pilot/description_files/urdf/g1_29dof.urdf"
        )

        full_model = pin.buildModelFromUrdf(urdf_path)

        lock = [
            "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
            "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
            "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
            "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
            "waist_yaw_joint","waist_roll_joint","waist_pitch_joint"
        ]

        lock_ids = [full_model.getJointId(j) for j in lock if full_model.existJointName(j)]
        self.model = pin.buildReducedModel(full_model, lock_ids, pin.neutral(full_model))
        self.data = self.model.createData()

        self.left_frame = self.model.getFrameId("left_rubber_hand")
        self.right_frame = self.model.getFrameId("right_rubber_hand")

        self.q = None

        self.t_left_p = np.zeros(3)
        self.t_right_p = np.zeros(3)
        self.t_left_rpy = np.zeros(3)
        self.t_right_rpy = np.zeros(3)

        self.arm_names_left = [
            "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
            "left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint"
        ]

        self.arm_names_right = [
            "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
            "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"
        ]

        self.g1_left = [15,16,17,18,19,20,21]
        self.g1_right = [22,23,24,25,26,27,28]

        self.v_left = [self.model.joints[self.model.getJointId(n)].idx_v for n in self.arm_names_left]
        self.v_right = [self.model.joints[self.model.getJointId(n)].idx_v for n in self.arm_names_right]

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = ('127.0.0.1', 9876)

        ChannelFactoryInitialize(1, "lo")
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.state_callback, 10)

        print("\n🎮 CONTROL FINAL")
        print("IZQ → WASD + RF | QETGZX")
        print("DER → IJKL + OU | NUM: 7/9 4/6 1/3")
        print("ESC salir\n")

        self.old = termios.tcgetattr(sys.stdin)

        threading.Thread(target=self.input_loop, daemon=True).start()
        threading.Thread(target=self.control_loop, daemon=True).start()

    def input_loop(self):

        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                if select.select([sys.stdin], [], [], 0)[0]:
                    k = sys.stdin.read(1)
                    if ord(k) == 27:
                        os._exit(0)
                    self.last_key = k
                    self.last_time = time.time()
                time.sleep(0.005)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old)

    def apply_input(self):

        if time.time() - self.last_time > 0.1:
            return
        
        k = (self.last_key or "").lower()
        dp = 0.01
        dr = 0.03

        # LEFT POS
        if k == 'w': self.t_left_p[0] += dp
        if k == 's': self.t_left_p[0] -= dp
        if k == 'a': self.t_left_p[1] += dp
        if k == 'd': self.t_left_p[1] -= dp
        if k == 'r': self.t_left_p[2] += dp
        if k == 'f': self.t_left_p[2] -= dp

        # RIGHT POS
        if k == 'i': self.t_right_p[0] += dp
        if k == 'k': self.t_right_p[0] -= dp
        if k == 'j': self.t_right_p[1] += dp
        if k == 'l': self.t_right_p[1] -= dp
        if k == 'o': self.t_right_p[2] += dp
        if k == 'u': self.t_right_p[2] -= dp

        # LEFT ORI
        if k == 'q': self.t_left_rpy[0] += dr
        if k == 'e': self.t_left_rpy[0] -= dr
        if k == 't': self.t_left_rpy[1] += dr
        if k == 'g': self.t_left_rpy[1] -= dr
        if k == 'z': self.t_left_rpy[2] += dr
        if k == 'x': self.t_left_rpy[2] -= dr

        # RIGHT ORI (NUM)
        if k == '7': self.t_right_rpy[0] += dr
        if k == '9': self.t_right_rpy[0] -= dr
        if k == '4': self.t_right_rpy[1] += dr
        if k == '6': self.t_right_rpy[1] -= dr
        if k == '1': self.t_right_rpy[2] += dr
        if k == '3': self.t_right_rpy[2] -= dr

    def ik(self, q, v_idx, frame, names, ids, p, rpy):

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        oMf = self.data.oMf[frame]

        err = np.hstack([
            p - oMf.translation,
            rpy - pin.rpy.matrixToRpy(oMf.rotation)
        ])

        J = pin.computeFrameJacobian(
            self.model, self.data, q, frame,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )[:, v_idx]

        dq = np.linalg.pinv(J) @ err

        qd = np.zeros(self.model.nv)
        for i, n in enumerate(names):
            j = self.model.getJointId(n)
            qd[self.model.joints[j].idx_v] = dq[i]

        q_next = pin.integrate(self.model, q, qd * self.dt)

        cmds = {}
        for i, n in enumerate(names):
            j = self.model.getJointId(n)
            cmds[ids[i]] = float(q_next[self.model.joints[j].idx_q])

        return q_next, cmds

    def control_loop(self):

        while True:

            if not self.state_received:
                time.sleep(self.dt)
                continue

            if not self.initialized:

                self.q = pin.neutral(self.model)

                pin.forwardKinematics(self.model, self.data, self.q)
                pin.updateFramePlacements(self.model, self.data)

                self.t_left_p = self.data.oMf[self.left_frame].translation.copy()
                self.t_right_p = self.data.oMf[self.right_frame].translation.copy()

                self.t_left_rpy = pin.rpy.matrixToRpy(self.data.oMf[self.left_frame].rotation)
                self.t_right_rpy = pin.rpy.matrixToRpy(self.data.oMf[self.right_frame].rotation)

                self.initialized = True
                print("[OK] Inicializado")

            self.apply_input()

            self.q, c1 = self.ik(self.q, self.v_left, self.left_frame,
                                 self.arm_names_left, self.g1_left,
                                 self.t_left_p, self.t_left_rpy)

            self.q, c2 = self.ik(self.q, self.v_right, self.right_frame,
                                 self.arm_names_right, self.g1_right,
                                 self.t_right_p, self.t_right_rpy)

            cmds = {**c1, **c2}

            try:
                self.sock.sendto(json.dumps(cmds).encode(), self.addr)
            except:
                pass

            time.sleep(self.dt)

    def state_callback(self, msg: LowState_):
        for i in range(29):
            self.current_jpos[i] = msg.motor_state[i].q
        self.state_received = True


if __name__ == "__main__":
    G1PerfectIK()
    while True:
        time.sleep(1)
