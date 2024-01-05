#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class MyNode(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.callback_image,
            10
        )
        self.image = None
        self.timer = self.create_timer(1 / 20, self.callback_timer)
        
    def callback_timer(self):
        if self.image is None: return
        frame = self.image.copy()
        cv2.imshow("frame", frame)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            raise SystemExit
            
    def callback_image(self, msg):
        self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")


def main(args=None):
    rclpy.init(args=args)
    
    node = MyNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
