#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def callback_image(msg):
    global image
    image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_depth(msg):
    global depth
    depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")
    
def get_depth(x, y):
    global depth
    if depth is None: return -1
    if depth[y][x] != 0: return depth[y][x]
    h, w = depth.shape[:2]
    for k in range(1, 10 + 1, 1):
        if y - k >= 0:
            for j in range(x - k, x + k + 1, 1):
                if x - k < 0: continue
                if x + k >= w: continue
                if depth[y - k][j] != 0: 
                    return depth[y - k][j]
        if x + k < w:
            for i in range(y - k, y + k + 1, 1):
                if y - k < 0: continue
                if y + k >= h: continue
                if depth[i][x + k] != 0:
                    return depth[i][x + k]
        if y + k < h:
            for j in range(x - k, x + k + 1, 1):
                if x - k < 0: continue
                if x + k >= w: continue
                if depth[y + k][j] != 0:
                    return depth[y + k][j]
        if x - k >= 0:
            for i in range(y - k, y + k + 1, 1):
                if y - k < 0: continue
                if y + k >= h: continue
                if depth[i][x - k] != 0:
                    return depth[i][x - k]
    return -1

def get_real_xyz(x, y):
    global depth
    h, w = depth.shape[:2]
    d = get_depth(x, y)
    a = 49.5 * np.pi / 180
    b = 60.0 * np.pi / 180
    real_y = (h / 2 - y) * 2 * d * np.tan(a / 2) / h
    real_x = (w / 2 - x) * 2 * d * np.tan(b / 2) / w
    return real_x, real_y, d
    

if __name__ == "__main__":
    rospy.init_node("demo")
    rospy.loginfo("demo start!")
    
    path = "/home/pcms/haarcascade_frontalface_default.xml"
    model = cv2.CascadeClassifier(path)
    
    image = None
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)
    rospy.wait_for_message("/camera/rgb/image_raw", Image)
    
    depth = None
    rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)
    rospy.wait_for_message("/camera/depth/image_raw", Image)
    
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        
        out = model.detectMultiScale(image)
        target = -1
        for i in range(len(out)):
            x, y, w, h = out[i]
            if target == -1 or out[target][2] * out[target][3] < w * h:
                target = i
        if target != -1:
            x, y, w, h = out[target]
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cx, cy = x + w // 2, y + h // 2
            cv2.circle(image, (cx, cy), 10, (0, 255, 0), -1)
            rospy.loginfo("XYZ: %d, %d, %d\n" % (get_real_xyz(cx, cy)))
        
        cv2.imshow("image", image)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break
        
    rospy.loginfo("demo end!")
    
