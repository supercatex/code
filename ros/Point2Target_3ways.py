#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np 
from pcms.openvino_models import Yolov8, HumanPoseEstimation

def callback_image(msg):
    global _image 
    _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_depth(msg):
    global _depth 
    _depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")

def get_xyz(x, y, depth):
    a = 55.0 * np.pi / 180
    b = 86.0 * np.pi / 180
    d = depth[y][x]
    h, w = depth.shape[:2]
    x = x - w // 2
    y = y - h // 2
    real_y = round(y * 2 * d * np.tan(a / 2) / h)
    real_x = round(x * 2 * d * np.tan(b / 2) / w)
    return real_x, real_y, d

def calc_distance(A, B, P):
    x1, y1, z1 = map(int, A)
    x2, y2, z2 = map(int, B) 
    x3, y3, z3 = map(int, P) 

    ''' method 3
    s = abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) * 0.5
    if x2 - x1 == 0 or y2 - y1 == 0 or z2 - z1 == 0: return 0
    h = 2 * s / (((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5)
    return h
    '''

    ''' method 2
    x3 -= x1
    y3 -= y1 
    z3 -= z1 
    x2 -= x1 
    y2 -= y1 
    z2 -= z1
    a = y3 * z2 - y2 * z3
    b = z3 * x2 - z2 * x3 
    c = x3 * y2 - x2 * y3 
    d = (a * a + b * b + c * c) ** 0.5
    if (x2 * x2 + y2 * y2 + z2 * z2) == 0: return 0
    d = d / ((x2 * x2 + y2 * y2 + z2 * z2) ** 0.5)
    return d
    '''

    # method 1
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
    #'''

if __name__ == "__main__":
    rospy.init_node("demo")

    _image, _depth = None, None
    rospy.Subscriber("/camera/color/image_raw", Image, callback_image)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)
    rospy.wait_for_message("/camera/color/image_raw", Image)
    rospy.wait_for_message("/camera/depth/image_raw", Image)

    dnn_yolo = Yolov8("yolov8n", device_name="GPU")
    dnn_pose = HumanPoseEstimation(device_name="GPU")

    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()

        frame = _image.copy()

        res = dnn_yolo.forward(_image)
        for x1, y1, x2, y2, s, c in res[0]["det"]:
            if s < 0.1: continue
            x1, y1, x2, y2, c = map(int, [x1, y1, x2, y2, c])
            if dnn_yolo.classes[c] != 'bottle': continue
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            rx, ry, d = get_xyz(cx, cy, _depth)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(frame, "%s %.2f" % (dnn_yolo.classes[c], s), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(frame, "%dmm" % (d), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        target_pose = None
        poses = dnn_pose.forward(_image)
        for i, pose in enumerate(poses):
            points = []
            for j, (x, y, preds) in enumerate(pose):
                if preds <= 0: continue
                if j in [8, 10]:
                    points.append(j)
            if len(points) == 2:
                target_pose = poses[i]
                break

        if target_pose is not None:
            x1, y1, _ = map(int, pose[10])
            x2, y2, _ = map(int, pose[8])
            
            A = get_xyz(x1, y1, _depth)
            B = get_xyz(x2, y2, _depth)
            
            cv2.circle(frame, (x1, y1), 10, (0, 255, 0), -1)
            cv2.circle(frame, (x2, y2), 10, (0, 255, 0), -1)
            cv2.putText(frame, "%d" % A[2], (x1, y1), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, "%d" % B[2], (x2, y2), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            for x1, y1, x2, y2, s, c in res[0]["det"]:
                if s < 0.1: continue
                x1, y1, x2, y2, c = map(int, [x1, y1, x2, y2, c])
                if dnn_yolo.classes[c] != 'bottle': continue
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                P = get_xyz(cx, cy, _depth)
                d = calc_distance(A, B, P)
                cv2.rectangle(frame, (x1, y1 - 70), (x2, y1 - 50), (0, 255, 0), -1)
                cv2.putText(frame, "%d" % d, (x1, y1 - 70), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("frame", frame)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break
