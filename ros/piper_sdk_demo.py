import time
from piper_sdk import *
import numpy as np 
import matplotlib.pyplot as plt
import random
import json


class PCMS_Piper(object):
    def __init__(self, port):
        super().__init__() 
        self.port = port 

    # https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD#connectport
    def connect(self):
        self.piper = C_PiperInterface_V2(self.port)
        self.piper.DisconnectPort()

        self.piper.ConnectPort()
        while not self.piper.EnablePiper():
            time.sleep(0.01)
        print(f"Port {self.port} connect success.")

    def emergency_stop(self):
        self.piper.EmergencyStop(0x01)

    def resume(self):
        self.piper.EmergencyStop(0x02)

    def reset(self):
        self.piper.ResetPiper()

    def init(self):
        self.piper.PiperInit()

    def move_check(self, joints):
        if len(joints) != 6:
            print("len(joints) should be 6.")
            return -1, []

        res = 0
        if not (-2.6179 <= joints[0] <= 2.6179):
            print("joint_1 angle problem:", joints[0], joints[0] * 180 / np.pi)
            joints[0] = max(-2.6179, min(joints[0], 2.6179))
            res = -2
        if not 0.0 <= joints[1] <= 3.14:
            print("joint_2 angle problem:", joints[1], joints[1] * 180 / np.pi)
            joints[1] = max(0.0, min(joints[1], 3.14))
            res = -2
        if not -2.967 <= joints[2] <= 0.0:
            print("joint_3 angle problem:", joints[2], joints[2] * 180 / np.pi)
            joints[2] = max(-2.967, min(joints[2], 0.0))
            res = -2
        if not -1.745 <= joints[3] <= 1.745:
            print("joint_4 angle problem:", joints[3], joints[3] * 180 / np.pi)
            joints[3] = max(-1.745, min(joints[3], 1.745))
            res = -2
        if not -1.22 <= joints[4] <= 1.22:
            print("joint_5 angle problem:", joints[4], joints[4] * 180 / np.pi)
            joints[4] = max(-1.22, min(joints[4], 1.22))
            res = -2
        if not -2.09439 <= joints[5] <= 2.09439:
            print("joint_6 angle problem:", joints[5], joints[5] * 180 / np.pi)
            joints[5] = max(-2.09439, min(joints[5], 2.09439))
            res = -2
        return res, joints

    # https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD#modectrl
    # https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD#jointctrl
    def move_joint(self, joints, speed=30, wait_util_finish=True):
        res, joints = self.move_check(joints)
        if res == -1: return -1
        print("move to", [round(i, 2) for i in joints])

        joints = [round(x * 1000 * 180 / np.pi) for x in joints]
        self.piper.ModeCtrl(0x01, 0x01, speed, 0x00)
        self.piper.JointCtrl(*joints)

        while wait_util_finish:
            time.sleep(0.005)
            js = self.piper.GetArmJointMsgs().joint_state
            jx = [js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6]
            max_e = np.inf 
            max_e = max([abs(x1 - x2) for x1, x2 in zip(jx, joints)])
            # print(max_e)
            if max_e < 500:
                wait_util_finish = False
        return res

    def calc_joints(self, x, y, z, rx, ry, rz):
        L0 = 123.0
        L1 = 285.0
        L2 = 255.0
        L3 = 90.0

        if not L1 + L2 >= (x ** 2 + y ** 2 + z ** 2) ** 0.5 >= 100:
            print("cannot reach that point!", x, y, z)
            return -1, []

        c = np.arctan2(z, x)
        x1 = (x * x + z * z) ** 0.5
        if x1 < 0: x1 = -x1
        y1 = y
        z1 = 0

        Lk = (x1 * x1 + y1 * y1) ** 0.5
        b = np.pi - np.arccos((L1 ** 2 + L2 ** 2 - Lk ** 2) / (2 * L1 * L2))
        a = np.pi / 2 - np.arctan2(y1, x1) - np.arccos((L1 ** 2 + Lk ** 2 - L2 ** 2) / (2 * L1 * Lk))
        a = a + np.pi / 2 - 8 * np.pi / 180
        b = b - np.pi + 17.5 * np.pi / 180

        print("angles:", [round(x * 180 / np.pi, 2) for x in [c, a, b]])
        x2 = x1 + L3 * np.cos(-(a + b))
        y2 = y1 + L3 * np.sin(-(a + b))
        z2 = z1

        x3 = x2 * np.cos(c) + z2 * np.sin(c)
        y3 = y2 
        z3 = x2 * np.sin(c) + z2 * np.cos(c)
        
        x4 = x + L3
        y4 = y
        z4 = z

        x5 = x1 * np.cos(c) + z1 * np.sin(c)
        y5 = y1 
        z5 = x1 * np.sin(c) + z1 * np.cos(c)

        print("x0:", [round(i, 2) for i in [x, y, z]])
        print("x1:", [round(i, 2) for i in [x1, y1, z1]])
        print("x2:", [round(i, 2) for i in [x2, y2, z2]])
        print("x3:", [round(i, 2) for i in [x3, y3, z3]])
        print("x4:", [round(i, 2) for i in [x4, y4, z4]])
        print("x5:", [round(i, 2) for i in [x5, y5, z5]])
        print("L3:", ((x3 - x) ** 2 + (y3 - y) ** 2 + (z3 - z) ** 2) ** 0.5)
        print("L4:", ((x4 - x) ** 2 + (y4 - y) ** 2 + (z4 - z) ** 2) ** 0.5)

        rx = self.calculate_rotation_angle(
            [x5, y5, z5],
            [x3, y3, z3],
            [x4, y4, z4]
        )
        ry = self.calculate_angle_B(
            [x3, y3, z3],
            [x5, y5, z5],
            [x4, y4, z4]
        )
        if abs(rx - np.pi) < abs(rx): 
            rx = rx - np.pi
            ry = -ry
        if z > 0: rx = -rx
        rz = -rx - 17 * np.pi / 180
        return 0, [c, a, b, rx, ry, rz]

    def move_point(self, x, y, z, rx, ry, rz, speed=30, wait_util_finish=True):
        res, joints = self.calc_joints(x, y, z, rx, ry, rz)
        return self.move_joint(joints, speed, wait_util_finish)


    def calculate_rotation_angle(self, A, B, C):
        O = np.array([0, 0, 0])
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        
        AB = B - A
        AB_norm = np.linalg.norm(AB)
        if AB_norm == 0:
            raise ValueError("A 點與 B 點重合，無法構成旋轉軸。")
        
        n1 = np.cross(A - O, B - O)
        n2 = np.cross(B - A, C - A)
        if np.linalg.norm(n1) == 0 or np.linalg.norm(n2) == 0:
            raise ValueError("無法構成平面（有點共線的情況）。")
        
        u = AB / AB_norm
        n1_perp = n1 - np.dot(n1, u) * u
        n2_perp = n2 - np.dot(n2, u) * u
        
        n1_perp_norm = np.linalg.norm(n1_perp)
        n2_perp_norm = np.linalg.norm(n2_perp)
        if n1_perp_norm == 0 or n2_perp_norm == 0:
            return 0.0
            
        cos_theta = np.dot(n1_perp, n2_perp) / (n1_perp_norm * n2_perp_norm)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        return np.arccos(cos_theta)

    def calculate_angle_B(self, A, B, C):
        A = np.array(A)
        B = np.array(B)  
        C = np.array(C)
        
        BA = A - B
        BC = C - B

        BA_norm = np.linalg.norm(BA)
        BC_norm = np.linalg.norm(BC)
        if BA_norm == 0 or BC_norm == 0:
            raise ValueError("A 或 C 不能與 B 重合")
        
        dot_product = np.dot(BA, BC)
        cos_B = dot_product / (BA_norm * BC_norm)
        cos_B = np.clip(cos_B, -1.0, 1.0)
        return np.arccos(cos_B)


if __name__ == "__main__":
    piper = PCMS_Piper("can0")
    piper.connect()

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim3d([-600, 600])
    ax.set_ylim3d([-600, 600])
    ax.set_zlim3d([-600, 600])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    px, py, pz, pc = [0], [0], [0], ["black"]
    d = 100
    for x in range(100, 601, d):
        for y in range(-600, 601, d):
            for z in range(-600, 601, d):
                res, joints = piper.cal_joints(x, y, z, 0, 0, 0)
                res, _ = piper.move_check(joints)
                
                if res == 0: 
                    px.append(x); py.append(y); pz.append(z);
                    pc.append("blue")
                # elif res == -1: 
                #     px.append(x); py.append(y); pz.append(z);
                #     pc.append("red")
                elif res == -2: 
                    px.append(x); py.append(y); pz.append(z);
                    pc.append("yellow")
    ax.scatter(px, py, pz, s=10, c=pc)
    plt.show()


    piper.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for x, y, z in zip(px, py, pz):
        piper.move_point(x, y, z, 0.0, 0.0, 0.0, 50)
        