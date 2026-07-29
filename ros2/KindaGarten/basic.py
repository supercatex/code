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


def get_real_xyz(x, y, depth, rpy=(0, 0, 0), k = 0):
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
    # x / w = real_x / real_w
    # real_w / 2 / d = tan(a / 2)
    real_x = 2 * d * np.tan(a / 2) * x / w 
    real_y = 2 * d * np.tan(b / 2) * y / h 
    # real_x = x * d / 620
    # real_y = y * d / 620
    real_x, real_y, real_z = d, -real_x, -real_y + 600

    rx, ry, rz = rpy
    ry = np.deg2rad(np.rad2deg(ry) + 21)
    # print(rpy)

    return real_x / 1000, real_y / 1000, real_z / 1000


if __name__ == "__main__":
    print(get_real_xyz(100, 200, np.zeros((720, 1280))))
