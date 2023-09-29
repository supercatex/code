#!/usr/bin/python
#coding=utf-8
import cv2 
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import pyzbar.pyzbar as pyzbar
import numpy as np

rospy.init_node('qrcode', anonymous=False)
qrdata_pub=rospy.Publisher("qrcode_data",String,queue_size=1)
qrimg_pub=rospy.Publisher("qrcode_img",Image,queue_size=10)
qrdata=String()
qrimg_input=np.zeros((640,480,3),np.uint8)
qrimg_output=Image()
cvb=CvBridge()

def callback(data):
    frame=cvb.imgmsg_to_cv2(data,"bgr8")
    global qrimg_input
    qrimg_input=frame
def get_img():
    rospy.Subscriber("camera/rgb/image_raw", Image, callback)
    
def qrcode_detect():
    img=qrimg_input.copy()
    #frame=cvb.imgmsg_to_cv2(qrimg_input,desired_encoding="passthrough")
    # 图像灰化，降低计算复杂度
    frame_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   
    barcodes = pyzbar.decode(frame_gray) 
    #将图片传给Zbar
    if len(barcodes)>0:
        for barcode in barcodes:
            barcodeData = barcode.data.decode("utf-8")
            left=barcode.rect.left
            top=barcode.rect.top
            width=barcode.rect.width
            height=barcode.rect.height
            cv2.rectangle(img,(left,top),((left+width),(top+height)),(0,255,0),2)
            if len(barcodeData) >= 1:
                qrdata_pub.publish(barcodeData)
    qrimg_output=cvb.cv2_to_imgmsg(img,"bgr8")
    qrimg_pub.publish(qrimg_output)
            


if __name__ == '__main__':
    print("二维码识别启动")
    try:
        get_img()
        while True:
            qrcode_detect()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("qrcode node terminated.")
