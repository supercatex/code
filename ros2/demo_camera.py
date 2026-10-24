import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
import cv2
import numpy as np
from ultralytics import YOLO
import os 
import time 


class SingleThreadedExecutorInstance(object):
    _instance: SingleThreadedExecutor = None
    def __new__(cls):
        if cls._instance is None:
            cls._instance = SingleThreadedExecutor()
        return cls._instance


class SubscriptionNode(Node):
    def __init__(self, msg_type: type, topic_name: str, func = None):
        super().__init__(topic_name.replace("/", "_"))
        self.msg_type = msg_type
        self.topic_name = topic_name
        self.func = func 

        self.data = None 
        self.sub = self.create_subscription(
            self.msg_type,
            self.topic_name,
            self.callback_data,
            qos_profile_sensor_data
        )
        SingleThreadedExecutorInstance().add_node(self)

    def callback_data(self, msg):
        if self.func is not None:
            self.data = self.func(msg)
        else:
            self.data = msg
    

class DepthCameraNode(Node):
    def __init__(self, topic_color, topic_depth):
        self.sub_image = SubscriptionNode(
            Image, 
            topic_color, 
            lambda x: CvBridge().imgmsg_to_cv2(x, "bgr8")
        )
        self.sub_depth = SubscriptionNode(
            Image, 
            topic_depth, 
            lambda x: CvBridge().imgmsg_to_cv2(x, "passthrough")
        )


class MainLoopNode(Node):
    def __init__(self):
        super().__init__("MainLoop")
        self.cam1 = DepthCameraNode("/camera/color/image_raw", "/camera/depth/image_raw")

    @property
    def image(self): return self.cam1.sub_image.data

    @property
    def depth(self): return self.cam1.sub_depth.data


if __name__ == "__main__":
    rclpy.init()
    node = MainLoopNode()

    model_name = "yolo26x-seg"
    if not os.path.exists(model_name + ".engine"):
        model = YOLO("yolo26x-seg.pt")
        model.export(format="engine", quantize=16)
    model = YOLO(model_name + ".engine")

    series = []

    while rclpy.ok():
        t1 = time.time()
        SingleThreadedExecutorInstance().spin_once()
        if node.image is None: continue 
        if node.depth is None: continue 

        depth_array = 255 - np.array(node.depth, dtype=np.float32) / np.max(node.depth) * 255
        depth_color = cv2.applyColorMap(cv2.convertScaleAbs(depth_array), cv2.COLORMAP_JET)

        res = model(node.image, verbose=False)[0]
        img = res.plot()
        img = cv2.resize(img, depth_color.shape[:2][::-1])
        frame = cv2.hconcat([img, depth_color])

        cv2.imshow("frame", frame)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break

        while len(series) > 0 and time.time() - series[0] > 1:
            series.pop(0)
        series.append(time.time())
        print("FPS:", len(series))

    rclpy.shutdown()
    
