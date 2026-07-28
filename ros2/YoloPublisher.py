import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import cv2 
import numpy as np 
import time 
from ultralytics import YOLO
import base64
import json 
import copy


class YoloPublisher(Node):
    def __init__(self):
        super().__init__("YoloPublisher")
        
        self.msg_image = None 
        self.msg_depth = None
        self.create_subscription(
            Image,
            "/ob_camera_head/color/image_raw",
            self.callback_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            "/ob_camera_head/depth/image_raw",
            self.callback_depth, qos_profile_sensor_data
        )
        self.pub_yolo = self.create_publisher(
            String, 
            "/ob_camera_head/yolo/obj", 10
        )
        self.pub_image = self.create_publisher(
            Image,
            "/ob_camera_head/yolo/image", 10
        )
        self.pub_depth = self.create_publisher(
            Image,
            "/ob_camera_head/yolo/depth", 10
        )

        self.yolo = YOLO("yolo26x-seg.pt")

    def callback_image(self, msg): self.msg_image = msg
    def callback_depth(self, msg): self.msg_depth = msg


if __name__ == "__main__":
    rclpy.init()
    node = YoloPublisher()
    while rclpy.ok():
        t1 = time.time()
        rclpy.spin_once(node)

        if node.msg_image is None or node.msg_depth is None:
            node.get_logger().warn("Image or depth not received.")
            time.sleep(3)
            continue 

        msg_image = copy.deepcopy(node.msg_image)
        msg_depth = copy.deepcopy(node.msg_depth)
        image = cv2.cvtColor(np.frombuffer(msg_image.data, dtype=np.uint8).reshape((msg_image.height, msg_image.width, 3)), cv2.COLOR_RGB2BGR)
        depth = np.frombuffer(msg_depth.data, dtype=np.uint16).reshape((msg_depth.height, msg_depth.width))
        image = np.array(image, dtype=np.uint8)
        depth = np.array(depth, dtype=np.int32)

        res = node.yolo.predict(image, device="0", conf=0.1, verbose=False)[0]
        
        data = {}
        data["obj"] = []
        if res.masks is not None:
            for c, xyxy, conf, xy in zip(res.boxes.cls, res.boxes.xyxy, res.boxes.conf, res.masks.xy):
                x1, y1, x2, y2 = map(int, xyxy)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cz = depth[cy][cx]
                data["obj"].append({
                    "cls": int(c),
                    "xyxy": [x1, y1, x2, y2],
                    "conf": float(conf),
                    "depth": int(cz),
                    "xy": xy.tolist()
                })
        else:
            for c, xyxy, conf in zip(res.boxes.cls, res.boxes.xyxy, res.boxes.conf):
                x1, y1, x2, y2 = map(int, xyxy)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cz = depth[cy][cx]
                data["obj"].append({
                    "cls": int(c),
                    "xyxy": [x1, y1, x2, y2],
                    "conf": float(conf),
                    "depth": int(cz)
                })

        json_string = json.dumps(data)
        msg = String()
        msg.data = json_string
        node.pub_yolo.publish(msg)
        node.pub_image.publish(msg_image)
        node.pub_depth.publish(msg_depth)

        time.sleep(max(0.0, min(0.05 - (time.time() - t1), 0.05)))
        node.get_logger().info(f"{time.time() - t1}")

    node.destroy_node()
    rclpy.shutdown()
