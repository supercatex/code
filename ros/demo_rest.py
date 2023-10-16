#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar
from RobotChassis import RobotChassis

def callback_image(msg):
    global _image
    _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def move_to(pos, next_status):
    global _robot, _status
    _robot.move_to(*pos)
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        if _robot.status_code == 3:
            _status = next_status
            break
            
if __name__ == "__main__":
    rospy.init_node("demo")
    rospy.loginfo("demo start!")
    
    # define global variables.
    _image = None    
    _robot = RobotChassis()
    _status = 0
    _pos_home = [0, 0, 0]
    _pos_tables = {
        "table1": [0, 0, 0],
        "table2": [0, 0, 0],
        "table3": [0, 0, 0],
        "table4": [0, 0, 0]
    }
    _orders = {
        "order1": "order1",
        "order2": "order2"
    }
    _curr_table = ""
    _curr_order = ""
    
    # define Subscriber and Publisher.
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)
    
    # main loop.
    rospy.loginfo("MAIN LOOP")
    rospy.sleep(1)
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        if _image is None: continue

        cv2.imshow("image", _image)
        cv2.waitKey(1)
        
        # detect QRCode
        qrcode = ""
        gray = cv2.cvtColor(_image, cv2.COLOR_BGR2GRAY)
        for barcode in pyzbar.decode(gray):
            qrcode = barcode.data.decode("UTF-8")
    
        if _status == 0:        # 等待指示進行帶位/送餐
            name = qrcode
            if name in _pos_tables:
                _curr_table = name
                rospy.loginfo("Moving to %s" % name)
                move_to(_pos_tables[name], 1)
                rospy.loginfo("Arrived %s" % name)
        elif _status == 1:      # 等待指示進行點餐
            name = qrcode
            if name in _orders:
                _curr_order = name
                rospy.loginfo("Received order %s" % name)
                move_to(_pos_home, 0)
                rospy.loginfo("Table %s ordered %s" % (_curr_table, _curr_order))
        else:
            pass

    rospy.loginfo("demo end!")
