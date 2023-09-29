#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def callback_image(msg):
    global image
    image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    rospy.loginfo("%d x %d" % (msg.width, msg.height))

if __name__ == "__main__":
    rospy.init_node("demo3")
    rospy.loginfo("demo3 start!")

    image = None
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)

    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        
        if image is None: continue
        cv2.imshow("image", image)
        cv2.waitKey(1)
