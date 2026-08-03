import rclpy
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from std_msgs.msg import String 
from rclpy.executors import SingleThreadedExecutor

import cv2
import numpy as np 
import time, base64, json, copy, requests
from ollama import Client


class SingleThreadedExecutorInstance(object):
    _instance: SingleThreadedExecutor = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = SingleThreadedExecutor()
        return cls._instance


class SubscriptionNode(Node):
    def __init__(self, msg_type, topic_name: str, func=None):
        super().__init__(topic_name.replace("/", "_"))
        self.msg_type = msg_type
        self.topic_name = topic_name 
        self.func = func

        self.data = None 
        self.sub = self.create_subscription(
            self.msg_type, self.topic_name,
            self.callback_data, qos_profile_sensor_data
        )
        SingleThreadedExecutorInstance().add_node(self)

    def callback_data(self, msg):
        if self.func is not None:
            self.data = self.func(msg)
        else:
            self.data = msg 


def get_real_xyz(x, y, depth, rpy=(0, 0, 0), k=0):
    d = depth[y][x]
    h, w = depth.shape
    for dx in range(-k, k + 1, 1):
        for dy in range(-k, k + 1, 1):
            if y + dy < 0 or y + dy >= h: continue 
            if x + dx < 0 or x + dx >= w: continue
            if depth[y + dy][x + dx] == 0: continue 
            if d == 0 or 0 < depth[y + dy][x + dx] < d:
                d = depth[y + dy][x + dx]
    x, y = x - w // 2, y - h // 2
    a, b = np.deg2rad(86), np.deg2rad(55)
    real_x = 2 * d * np.tan(a / 2) * x / w 
    real_y = 2 * d * np.tan(b / 2) * y / h 
    real_x, real_y, real_z = d, -real_x, -real_y

    # rx, ry, rz = rpy
    # ry = np.deg2rad(np.rad2deg(ry) + 21)
    # new_z = (real_x - real_z * np.tan(ry)) * np.sin(ry) + real_z / np.cos(ry)
    # new_x = real_z * np.tan(ry) * np.cos(ry)
    # real_x, real_z = new_x, new_z

    return real_x / 1000, real_y / 1000, (real_z + 600) / 1000


def get_real_xyz_v2(x, y, depth, rpy=(0, 0, 0), k=0):
    d = depth[y][x]
    h, w, = depth.shape 
    for dx in range(-k, k + 1, 1):
        for dy in range(-k, k + 1, 1):
            if y + dy < 0 or y + dy >= h: continue 
            if x + dx < 0 or x + dx >= w: continue
            if depth[y + dy][x + dx] == 0: continue 
            if d == 0 or 0 < depth[y + dy][x + dx] < d:
                d = depth[y + dy][x + dx]
    x, y = x - w // 2, y - h // 2
    
    a, b = np.deg2rad(86), np.deg2rad(55)
    real_x = 2 * d * np.tan(a / 2) * x / w 
    real_y = 2 * d * np.tan(b / 2) * y / h 
    pos = np.array([d, -real_x, -real_y], dtype=np.float32)

    rpy = np.array(rpy, dtype=np.float32) + (0.0, np.deg2rad(30), 0.0)
    roll, pitch, yaw = rpy
    pos = pos / 1000 + (0.08, 0.00, 0.08)
    pos = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll) @ pos
    pos = pos + (0.0, 0.0, 0.64)
    return pos

def rot_x(roll):
    c = np.cos(roll)
    s = np.sin(roll)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c  , -s ],
        [0.0, s  ,  c ]
    ], dtype=np.float64)

def rot_y(pitch):
    c = np.cos(pitch)
    s = np.sin(pitch)
    return np.array([
        [c  , 0.0, s  ],
        [0.0, 1.0, 0.0],
        [-s , 0.0, c  ]
    ], dtype=np.float64)

def rot_z(yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([
        [c  , -s , 0.0],
        [s  , c  , 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)


if __name__ == "__main__":
    print(get_real_xyz(100, 200, np.zeros((720, 1280))))
