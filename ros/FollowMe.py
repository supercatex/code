#!/usr/bin/env python3
from typing import Tuple, List
import numpy as np


class FollowMe(object):
    def __init__(self) -> None:
        self.pre_x, self.pre_z = 0.0, 0.0

    def get_xy_by_pose(self, pose) -> Tuple[int, int]:
        if pose is None: return -1, -1

        p = []
        for i in [5, 6, 11, 12]:
            if pose[i][2] > 0:
                p.append(pose[i])
        if len(p) == 0: return -1, -1
        
        min_x = max_x = p[0][0]
        min_y = max_y = p[0][1]
        for i in range(len(p)):
            min_x = min(min_x, p[i][0])
            max_x = max(max_x, p[i][0])
            min_y = min(min_y, p[i][1])
            max_y = max(max_y, p[i][1])
        
        cx = int(min_x + max_x) // 2
        cy = int(min_y + max_y) // 2
        return cx, cy

    def get_target_pose(self, depth, poses) -> int:
        target = -1
        target_d = 9999999
        for i, pose in enumerate(poses):
            cx, cy = self.get_xy_by_pose(pose)
            _, _, d = self.get_real_xyz(depth, cx, cy)
            if target == -1 or (d != 0 and d < target_d):
                target = i
                target_d = d
        if target == -1: return None
        return poses[target]

    def get_real_xyz(self, depth, x: int, y: int) -> Tuple[float, float, float]:
        if x < 0 or y < 0: return 0, 0, 0

        a = 49.5 * np.pi / 180
        b = 60.0 * np.pi / 180
        d = depth[y][x]
        h, w = depth.shape[:2]
        x = x - w // 2
        y = y - h // 2
        real_y = y * 2 * d * np.tan(a / 2) / h
        real_x = x * 2 * d * np.tan(b / 2) / w
        return real_x, real_y, d

    def calc_linear_x(self, cd: float, td: float) -> float:
        if cd <= 0: return 0
        e = cd - td
        p = 0.0005
        x = p * e
        if x > 0: x = min(x, 0.5)
        if x < 0: x = max(x, -0.5)
        return x

    def calc_angular_z(self, cx: float, tx: float) -> float:
        if cx < 0: return 0
        e = tx - cx
        p = 0.0025
        z = p * e
        if z > 0: z = min(z, 0.3)
        if z < 0: z = max(z, -0.3)
        return z

    def calc_cmd_vel(self, image, depth, poses) -> Tuple[float, float]:
        image = image.copy()
        depth = depth.copy()
        
        pose = self.get_target_pose(depth, poses)
        cx, cy = self.get_xy_by_pose(pose)
        _, _, d = self.get_real_xyz(depth, cx, cy)

        cur_x = self.calc_linear_x(d, 1000)
        cur_z = self.calc_angular_z(cx, 320)

        dx = cur_x - self.pre_x
        if dx > 0: dx = min(dx, 0.05)
        if dx < 0: dx = max(dx, -0.05)
        
        dz = cur_z - self.pre_z
        if dz > 0: dz = min(dz, 0.1)
        if dz < 0: dz = max(dz, -0.1)

        cur_x = self.pre_x + dx
        cur_z = self.pre_z + dz

        self.pre_x = cur_x 
        self.pre_z = cur_z 

        return cur_x, cur_z


if __name__ == "__main__":
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    from pcms.openvino_models import HumanPoseEstimation
    import numpy as np
    from geometry_msgs.msg import Twist


    def callback_image(msg):
        global _image
        _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        

    def callback_depth(msg):
        global _depth
        _depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")
    

    rospy.init_node("FollowMe")
    rospy.loginfo("FollowMe started!")
    
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
    
    # cmd_vel Publisher
    _msg_cmd = Twist()
    _pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # Models
    _net_pose = HumanPoseEstimation()

    # Functions
    _fw = FollowMe()
    
    # Main loop
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        image = _image.copy()
        depth = _depth.copy()
        
        poses = _net_pose.forward(image)
        x, z = _fw.calc_cmd_vel(image, depth, poses)

        _msg_cmd.linear.x = x 
        _msg_cmd.angular.z = z
        _pub_cmd.publish(_msg_cmd)
        
        cv2.imshow("frame", image)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break
        
    rospy.loginfo("FollowMe end!")
