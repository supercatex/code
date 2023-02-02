#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pcms.openvino_models import HumanPoseEstimation
from pcms.pytorch_models import Yolov5
import numpy as np


def callback_image(msg):
    global _image
    _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    

def callback_depth(msg):
    global _depth
    _depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")
    

def get_real_xyz(x, y):
    global _depth
    depth = _depth.copy()
    a = 49.5 * np.pi / 180
    b = 60.0 * np.pi / 180
    d = depth[y][x]

    h, w = depth.shape[:2]
    if d == 0:
        for k in range(1, 15, 1):
            if d == 0 and y - k >= 0:
                for j in range(x - k, x + k, 1):
                    d = depth[y - k][j]
                    if d > 0: break
            if d == 0 and x + k < w:
                for i in range(y - k, y + k, 1):
                    d = depth[i][x + k]
                    if d > 0: break
            if d == 0 and y + k < h:
                for j in range(x + k, x - k, -1):
                    d = depth[y + k][j]
                    if d > 0: break
            if d == 0 and x - k >= 0:
                for i in range(y + k, y - k, -1):
                    d = depth[i][x - k]
                    if d > 0: break
            if d > 0: break    
    
    x = x - w // 2
    y = y - h // 2
    real_y = round(y * 2 * d * np.tan(a / 2) / h)
    real_x = round(x * 2 * d * np.tan(b / 2) / w)
    return real_x, real_y, d


def calc_distance(A, B, P):
    x1, y1, z1 = map(int, A)
    x2, y2, z2 = map(int, B) 
    x3, y3, z3 = map(int, P) 

    a = x2 - x1
    b = y2 - y1 
    c = z2 - z1

    t1 = a * x3 + b * y3 + c * z3
    t2 = a * x1 + b * y1 + c * z1
    t3 = a * a + b * b + c * c
    if t3 == 0: return 0
    t = (t1 - t2) / t3

    x4 = a * t + x1 
    y4 = b * t + y1
    z4 = c * t + z1

    dx = x4 - x3
    dy = y4 - y3
    dz = z4 - z3
    return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5


if __name__ == "__main__":
    rospy.init_node("demo3")
    rospy.loginfo("demo3 start!")
    
    # RGB Image Subscriber
    _image = None
    _topic_image = "/camera/rgb/image_raw"
    rospy.Subscriber(_topic_image, Image, callback_image)
    rospy.wait_for_message(_topic_image, Image)
    
    # Depth Image Subscriber
    _depth = None
    _topic_depth = "/camera/depth/image_raw"
    rospy.Subscriber(_topic_depth, Image, callback_depth)
    rospy.wait_for_message(_topic_depth, Image)

    # Models
    net_pose = HumanPoseEstimation()
    net_yolo = Yolov5()
    
    # Main loop
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        image = _image.copy()

        poses = net_pose.forward(image)
        if len(poses) > 0:
            if poses[0][9][2] > 0 and poses[0][7][2] > 0:
                A = list(map(int, poses[0][9][:2]))
                B = list(map(int, poses[0][7][:2]))
                cv2.circle(image, A, 3, (0, 255, 0), -1)
                cv2.circle(image, B, 3, (0, 255, 0), -1)
                A = get_real_xyz(*A)
                B = get_real_xyz(*B)

                objects = net_yolo.forward(image)
                for _, i, _, x1, y1, x2, y2 in objects:
                    if net_yolo.labels[i] == "bottle":
                        x, y = (x1 + x2) // 2, (y1 + y2) // 2
                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(image, (x, y), 3, (0, 255, 0), -1)
                        
                        P = get_real_xyz(x, y)
                        d = calc_distance(A, B, P)
                        cv2.putText(image, "%d" % d, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("image", image)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break

    rospy.loginfo("demo3 end!")
