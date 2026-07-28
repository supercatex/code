import rclpy
from rclpy.node import Node 
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
from std_msgs.msg import String 
from rclpy.executors import SingleThreadedExecutor

import cv2
import numpy as np 
import time, base64, json, copy 


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
