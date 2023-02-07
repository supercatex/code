#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from pcms.openvino_models import HumanPoseEstimation
from pcms.pytorch_models import Yolov5, FasterRCNN


def callback_image(msg):
    global _image
    _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")


def callback_depth(msg):
    global _depth
    _depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")


def get_xyz(x, y, depth):
    a = 49.5 * np.pi / 180
    b = 60.0 * np.pi / 180
    d = depth[y][x]

    h, w = depth.shape[:2]
    if d == 0:
        for k in range(1, 15, 1):
            if d == 0 and y - k >= 0:
                for j in range(x - k, x + k, 1):
                    if not (0 <= j < w): continue
                    d = depth[y - k][j]
                    if d > 0: break
            if d == 0 and x + k < w:
                for i in range(y - k, y + k, 1):
                    if not (0 <= i < h): continue
                    d = depth[i][x + k]
                    if d > 0: break
            if d == 0 and y + k < h:
                for j in range(x + k, x - k, -1):
                    if not (0 <= j < w): continue
                    d = depth[y + k][j]
                    if d > 0: break
            if d == 0 and x - k >= 0:
                for i in range(y + k, y - k, -1):
                    if not (0 <= i < h): continue
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
    rospy.init_node("demo")
    rospy.loginfo("demo start!")
    
    _image = None
    _image_topic = "/camera/rgb/image_raw"
    rospy.Subscriber(_image_topic, Image, callback_image)
    rospy.wait_for_message(_image_topic, Image)
    
    _depth = None
    _depth_topic = "/camera/depth/image_raw"
    rospy.Subscriber(_depth_topic, Image, callback_depth)
    rospy.wait_for_message(_depth_topic, Image)
    
    _net_objs = Yolov5()
    _net_poses = HumanPoseEstimation()
    
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        image = _image.copy()
        frame = _image.copy()
        depth = _depth.copy()
        
        h, w, c = image.shape
        h, w = h * 2, w * 2
        image = cv2.resize(image, (w, h))
        frame = cv2.resize(frame, (w, h))
        #depth = cv2.resize(depth, (w, h))
        
        objs = _net_objs.forward(image)
        for id, index, preds, x1, y1, x2, y2 in objs:
            if _net_objs.labels[index] != "bottle":
                continue
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            _, _, d = get_xyz(cx // 2, cy // 2, depth)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
            cv2.rectangle(frame, (x1, y1 - 50), (x2, y1), (0, 255, 0), -1)     
            cv2.putText(frame, "%d" % d, (x1, y1 - 20), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2, cv2.LINE_AA)
            
        pose = None
        poses = _net_poses.forward(image)
        for i, pose in enumerate(poses):
            points = []
            for j, (x, y, preds) in enumerate(pose):
                if preds <= 0: continue
                x, y = map(int, [x, y])
                if j in [8, 10]:
                    points.append(j)
            if len(points) == 2:
                pose = poses[i]
                break
        
        if pose is not None:
            x1, y1, _ = map(int, pose[10])
            x2, y2, _ = map(int, pose[8])
            
            A = get_xyz(x1 // 2, y1 // 2, depth)
            B = get_xyz(x2 // 2, y2 // 2, depth)
            
            cv2.circle(frame, (x1, y1), 10, (0, 255, 0), -1)
            cv2.circle(frame, (x2, y2), 10, (0, 255, 0), -1)
            cv2.putText(frame, "%d" % A[2], (x1, y1), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, "%d" % B[2], (x2, y2), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            for id, index, preds, x1, y1, x2, y2 in objs:
                if _net_objs.labels[index] != "bottle":
                    continue
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                P = get_xyz(cx // 2, cy // 2, depth)
                d = calc_distance(A, B, P)
                cv2.rectangle(frame, (x1, y1 - 70), (x2, y1 - 50), (0, 255, 0), -1)
                cv2.putText(frame, "%d" % d, (x1, y1 - 70), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2, cv2.LINE_AA)
                
                
        cv2.imshow("image", frame)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break
            
    rospy.loginfo("demo end!")
    
