import time
from piper_sdk import *
import numpy as np 
import matplotlib.pyplot as plt
import random
import json


# https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD
class KindaPiper(object):
    L0 = 123.00
    L1 = 285.00
    L2 = 255.50
    L3 =  90.00
    JOINTS_LIMIT = (
        (-2.6179, 2.6179),
        (0.0, 3.14),
        (-2.967, 0.0),
        (-1.745, 1.745),
        (-1.22, 1.22),
        (-2.09439, 2.09439)
    )

    def __init__(self, port="can0"):
        super().__init__()
        self.port = port 

    def connect(self, port=None):
        if port is not None: self.port = port 
        self.piper = C_PiperInterface_V2(self.port)
        # self.piper.DisconnectPort()

        self.piper.ConnectPort()
        while not self.piper.EnablePiper():
            time.sleep(0.01)
        print(f"Port {self.port} connect success.")

        self.piper.EnableArm(7, 0x02)

    def emergency_stop(self):
        self.piper.EmergencyStop(0x01)

    def resume(self):
        self.piper.EmergencyStop(0x02)

    def reset(self):
        self.piper.ResetPiper()

    def init(self):
        self.piper.PiperInit()

    def check_joints(self, joints):
        if len(joints) != 6:
            print("len(joints) should be 6.")
            return -1, []

        JOINTS_LIMIT = self.JOINTS_LIMIT
        angles = [round(i * 180 / np.pi, 2) for i in joints]
        res = 0
        for i in range(6):
            if not (JOINTS_LIMIT[i][0] <= joints[i] <= JOINTS_LIMIT[i][1]):
                print(f"joint_{i+1} angle problem:", joints[i], angles[i])
                joints[i] = max(JOINTS_LIMIT[i][0], min(joints[i], JOINTS_LIMIT[i][1]))
                res = -2
        return res, joints

    def move_joint(self, joints, speed=30, wait_util_finish=True):
        res, joints = self.check_joints(joints)
        if res == -1: return -1
        # print("move to", [round(i, 2) for i in joints])

        joints = [round(x * 1000 * 180 / np.pi) for x in joints]
        self.piper.ModeCtrl(0x01, 0x01, speed, 0x00)
        self.piper.JointCtrl(*joints)

        while wait_util_finish:
            time.sleep(0.01)
            js = self.piper.GetArmJointMsgs().joint_state
            jx = [js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6]
            max_e = np.inf 
            max_e = max([abs(x1 - x2) for x1, x2 in zip(jx, joints)])
            if max_e < 1000:
                wait_util_finish = False
        return res

    def calc_joints(self, x, y, z, rx, ry, rz):
        L0 = self.L0
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3

        if not (L1 + L2) >= (x ** 2 + y ** 2 + z ** 2) ** 0.5 >= 150:
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

        # print("angles:", [round(x * 180 / np.pi, 2) for x in [c, a, b]])
        x2 = x1 + L3 * np.cos(-(a + b))
        y2 = y1 + L3 * np.sin(-(a + b))
        z2 = z1

        x3 = x2 * np.cos(c) + z2 * np.sin(c)
        y3 = y2 
        z3 = x2 * np.sin(c) + z2 * np.cos(c)
        
        x4 = x + L3 * np.cos(rz) * np.cos(ry)
        z4 = z + L3 * np.cos(rx) * np.sin(ry)
        y4 = y + L3 * np.cos(rx) * np.sin(rz)
        # print(x, y, z, rx, ry, rz)
        # print(x4, y4, z4)
        # input()

        x5 = x1 * np.cos(c) + z1 * np.sin(c)
        y5 = y1 
        z5 = x1 * np.sin(c) + z1 * np.cos(c)

        # print("x0:", [round(i, 2) for i in [x, y, z]])
        # print("x1:", [round(i, 2) for i in [x1, y1, z1]])
        # print("x2:", [round(i, 2) for i in [x2, y2, z2]])
        # print("x3:", [round(i, 2) for i in [x3, y3, z3]])
        # print("x4:", [round(i, 2) for i in [x4, y4, z4]])
        # print("x5:", [round(i, 2) for i in [x5, y5, z5]])
        # print("L3:", ((x3 - x) ** 2 + (y3 - y) ** 2 + (z3 - z) ** 2) ** 0.5)
        # print("L4:", ((x4 - x) ** 2 + (y4 - y) ** 2 + (z4 - z) ** 2) ** 0.5)

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
        if z4 > 0: rx = -rx
        rz = -rx + np.sin(c) * 1.0
        return 0, [c, a, b, rx, ry, rz]

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

    def move_point(self, x, y, z, rx, ry, rz, speed=30, wait_util_finish=True):
        res, joints = self.calc_joints(x, y, z, rx, ry, rz)
        # print(joints)
        return self.move_joint(joints, speed, wait_util_finish)

    def mdh_matrix(self, alpha_prev, a_prev, d_i, theta_i):
        ct = np.cos(theta_i)
        st = np.sin(theta_i)
        ca = np.cos(alpha_prev)
        sa = np.sin(alpha_prev)
        
        return np.array([
            [ct,        -st,         0,   a_prev],
            [st * ca,    ct * ca,   -sa,  -sa * d_i],
            [st * sa,    ct * sa,    ca,   ca * d_i],
            [0,          0,          0,   1]
        ])

    def forward_kinematics(self, joint_angles):
        mdh_params = [
            [0,         0,         self.L0,   0],
            [-np.pi/2,  0,         0,       -172.22 / 180 * np.pi],
            [0,         self.L1,   0,       -102.78 / 180 * np.pi],
            [np.pi/2,   -0.021984, self.L2, 0],
            [-np.pi/2,  0,         0,       0],
            [np.pi/2,   0,         self.L3,   0]
        ]

        joint_radians = [i * np.pi / 180 for i in joint_angles]

        T_base_to_end = np.identity(4)
        for i in range(6):
            alpha_prev, a_prev, d_i, offset = mdh_params[i]
            theta_i = joint_radians[i] + offset
            T_joint = self.mdh_matrix(alpha_prev, a_prev, d_i, theta_i)
            T_base_to_end = np.dot(T_base_to_end, T_joint)
        pos = T_base_to_end[0:3, 3]
        rot = T_base_to_end[0:3, 0:3]
        roll, pitch, yaw = self.extract_euler_angles(rot)
        return pos, (roll, pitch, yaw) 

    def extract_euler_angles(self, R):
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        return roll, pitch, yaw

    def move_to_endpoint(self, x, y, z, rx, ry, rz, speed=30, wait_util_finish=True):
        x1 = x - self.L3 * np.cos(rz) * np.cos(ry)
        z1 = z - self.L3 * np.cos(rx) * np.sin(ry)
        y1 = y - self.L3 * np.cos(rx) * np.sin(rz)
        # print(x1, y1, z1)
        self.move_point(x1, y1, z1, rx, ry, rz, speed, wait_util_finish)
        return x, y, z

    def close_gripper(self, gripper=50):
        gripper *= 1000

        c = 0
        while True:
            time.sleep(0.005)
            msg = self.piper.GetArmGripperMsgs()
            angle = msg.gripper_state.grippers_angle
            effort = msg.gripper_state.grippers_effort

            # print(angle, effort)
            d = gripper - angle
            if abs(d) < 1000: 
                print("Gripper POSITION RREAK.")
                break

            k = round(angle + d * 0.5)
            if abs(k - gripper) < 1000:
                k = gripper

            if abs(effort) > 500:
                c += 1
            else:
                c = 0
            if c > 20:
                self.piper.GripperCtrl(angle - 100, 0, 0x01, 0x00)
                print("Gripper EFFORT BREAK.")
                break
            self.piper.GripperCtrl(k, 0, 0x01, 0x00)

    def move_point22(self, x, y, z, rx, ry, rz, speed=50):
        while True:
            time.sleep(0.005)

            factor = 1000
            position = [x, y, z, rx, ry, rz]
            X = round(position[0] * factor)
            Y = round(position[1] * factor)
            Z = round(position[2] * factor)
            RX = round(position[3] * factor)
            RY = round(position[4] * factor)
            RZ = round(position[5] * factor)
            self.piper.MotionCtrl_2(0x01, 0x00, speed, 0x00)
            self.piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
            time.sleep(0.005)
            # print(self.piper.GetArmEndPoseMsgs())
            # print(self.piper.GetArmStatus())
            status = self.piper.GetArmStatus().arm_status.arm_status
            # print("S:", status)
            if status == 0x04:
                print(f"POSITION({X},{Y},{Z}) ERROR!")
                # input()
                break

            end_pose = self.piper.GetArmEndPoseMsgs().end_pose
            cX = end_pose.X_axis
            cY = end_pose.Y_axis
            cZ = end_pose.Z_axis
            cRX = end_pose.RX_axis
            cRY = end_pose.RY_axis
            cRZ = end_pose.RZ_axis

            # print(X, Y, Z)
            # print(cX, cY, cZ)
            e1 = ((X - cX) ** 2 + (Y - cY) ** 2 + (Z - cZ) ** 2) ** 0.5
            # print("E1:", e1)
            if e1 < 1000:
                print(f"POSITION({X},{Y},{Z}) OK!")
                # input()
                break

    def get_end_pose(self):
        end_pose = self.piper.GetArmEndPoseMsgs().end_pose
        cX = end_pose.X_axis
        cY = end_pose.Y_axis
        cZ = end_pose.Z_axis
        cRX = end_pose.RX_axis
        cRY = end_pose.RY_axis
        cRZ = end_pose.RZ_axis
        return [cX, cY, cZ, cRX, cRY, cRZ]

    def calc_error(self, p1, p2):
        return sum([(i - j) ** 2 for i, j in zip(p1, p2)]) ** 0.5


if __name__ == "__main__":
    piper = KindaPiper()
    piper.connect("can0")

    piper.move_joint([0, 0, 0, 0, 0, 0], 50)


    # piper.piper.GripperTeachingPendantParamConfig(100, 70, 1)
    # piper.piper.ArmParamEnquiryAndConfig(4)

    # piper.piper.ModeCtrl(0x01, 0x01, 100, 0x00)

    # piper.piper.GripperCtrl(0, 1000, 0x02, 0)
    # piper.piper.GripperCtrl(0, 1000, 0x01, 0x00)
    # count = 0
    # k = 2
    # while True:
    #     time.sleep(0.005)
    #     count = (count + 1) % 1000
    #     if count == 0:
    #         if k == 2: 
    #             k = 6 
    #         else: 
    #             k = 2

    #     # piper.piper.ModeCtrl(0x01, 0x02, 100, 0x00)
    #     piper.piper.GripperCtrl(round(0.01 * 1000 * 1000) * k, 1000, 0x01, 0x00)

    #     print(piper.piper.GetArmGripperMsgs())
    #     print(piper.piper.GetGripperTeachingPendantParamFeedback())
    #     print(piper.piper.GetArmGripperCtrl())

    # exit(1)

    piper.close_gripper(70)
    piper.move_to_endpoint(300, 100, 0, 0, 0, 0, 50)
    input()
    piper.move_to_endpoint(450, 100, 0, 0, 0, 0, 50)
    piper.close_gripper(5)
    input()
    piper.move_to_endpoint(450, 200, 0, 0, 0, 0, 50)
    piper.move_to_endpoint(300, 200, 0, 0, 0, 0, 50)
    input()
    piper.move_to_endpoint(450, 100, 0, 0, 0, 0, 50)
    piper.close_gripper(70)
    exit(1)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim3d([-600, 600])
    ax.set_ylim3d([-600, 600])
    ax.set_zlim3d([-600, 600])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    px, py, pz, pc, pi = [0], [0], [0], ["black"], [0]
    d = 50
    for x in range(300, 551, d):
        for y in range(-600, 601, d):
            for z in range(-600, 601, d):
                res, joints = piper.calc_joints(x, y, z, 0, 0, 0)
                res, _ = piper.check_joints(joints)
                
                if res == 0: 
                    px.append(x); py.append(y); pz.append(z);
                    pc.append("blue")
                    pi.append(pi[-1] + 1)
                # elif res == -1: 
                #     px.append(x); py.append(y); pz.append(z);
                #     pc.append("red")
                    # pi.append(pi[-1] + 1)
                elif res == -2: 
                    px.append(x); py.append(y); pz.append(z);
                    pc.append("yellow")
                    pi.append(pi[-1] + 1)
    ax.scatter(px, py, pz, s=10, c=pc)
    # plt.show()

    # random.shuffle(pi)
    # piper.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for i in pi:
        # piper.move_point(px[i], py[i], pz[i], 0.0, 0.0, -np.pi / 4 * 0, 80)
        print("TARGET:", px[i], py[i], pz[i])
        # piper.move_to_endpoint(px[i], py[i], pz[i], 0.0, 0, -90*np.pi/180, 50)
        # time.sleep(1)
        # piper.move_to_endpoint(px[i], py[i], pz[i], 0.0, 0, -60*np.pi/180, 50)
        # time.sleep(1)
        # piper.move_to_endpoint(px[i], py[i], pz[i], 0.0, 0, -60*np.pi/180, 50)
        # time.sleep(1)
        # piper.move_to_endpoint(px[i], py[i], pz[i], 0.0, 0, -30*np.pi/180, 50)
        # time.sleep(1)


        piper.move_to_endpoint(px[i], py[i], pz[i], 0.0, 0, 0, 80)
        time.sleep(0.1)
        input()
        # piper.move_point22(px[i], py[i], pz[i], 0.0, 85, 0.0, 80)
        # time.sleep(0.1)
        # input()


        # piper.move_to_endpoint(px[i], py[i], pz[i], 0.0, 0, 30*np.pi/180, 50)
        # time.sleep(1)
        # piper.move_to_endpoint(px[i], py[i], pz[i], 0.0, 0, 60*np.pi/180, 50)
        # time.sleep(1)
        # input()
        # js = piper.piper.GetArmJointMsgs().joint_state
        # jx = [js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6]
        # jx = [i / 1000 for i in jx]
        # print(jx)
        # piper.forward_kinematics(jx)
        # print(px[i], pz[i], py[i])
        # input()
