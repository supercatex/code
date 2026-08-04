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


def find_depth(x, y, depth, w=0, h=0):
    x1, x2 = x - w // 2, x + w // 2 + 1
    y1, y2 = y - h // 2, y + h // 2 + 1
    m = np.array(depth[y1:y2, x1:x2], np.int32)
    m = np.where(m == 0, np.inf, m)
    d = np.min(m)
    return d 


def get_real_xyz_base_on_camera(x, y, depth, w=0, h=0):
    d = find_depth(x, y, depth, w, h)
    h, w = depth.shape
    x, y = x - w // 2, y - h // 2

    a, b = np.deg2rad(90), np.deg2rad(55)
    real_x = 2 * d * np.tan(a / 2) * x / w 
    real_y = 2 * d * np.tan(b / 2) * y / h 
    pos = np.array([d, -real_x, -real_y], dtype=np.float32)
    pos = pos / 1000
    return pos 


def get_real_xyz(x, y, depth, w=0, h=0):
    pos = get_real_xyz_base_on_camera(x, y, depth, w, h)
    pos = pos + (0.0, 0.0, 0.6)
    return pos


def get_real_xyz(x, y, depth, w=0, h=0, rpy=(0, 0, 0)):
    pos = get_real_xyz_base_on_camera(x, y, depth, w, h)

    rpy = np.array(rpy, dtype=np.float32) + (0.0, np.deg2rad(30), 0.0)
    roll, pitch, yaw = rpy
    pos = pos + (0.08, 0.015, 0.08)
    pos = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll) @ pos

    pos = pos + (0.0, 0.0, 0.6)
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
