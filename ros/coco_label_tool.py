#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


_DATASET_DIR = "/home/pcms/"
_DATASET_NAME = "my_dataset"
_DATASET_YAML = _DATASET_NAME + ".yaml"
_CLASSES = ["0", "1", "2", "3"]
_MODEL_BASE = 'yolov8n'
_EPOCHES = 500


def callback_image(msg):
    global _image
    _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")


def callback_mouse(event, x, y, flags, param):
    global _class_id, _x1, _y1, _x2, _y2, _status, _train
    if event == cv2.EVENT_LBUTTONDOWN:
        _x1, _y1 = x, y
        _x2, _y2 = -1, -1
        _status = 1
    if event == cv2.EVENT_MOUSEMOVE:
        if _status == 1:
            _x2, _y2 = x, y
    if event == cv2.EVENT_LBUTTONUP:
        _status = 0
    if event == cv2.EVENT_LBUTTONDBLCLK:
        _train = not _train
    if event == cv2.EVENT_MBUTTONUP:
        save()


def save():
    global train_images, train_labels, valid_images, valid_labels
    global _class_id, _x1, _y1, _x2, _y2, _train
    s = datetime.today().strftime('%Y%m%d_%H%M%S')
    if _train:
        images = train_images
        labels = train_labels
    else:
        images = valid_images
        labels = valid_labels
    cv2.imwrite(os.path.join(images, s + ".jpg"), image)
    with open(os.path.join(labels, s + ".txt"), "w") as f:
        x, y, w, h = _x1 / fw, _y1 / fh, (_x2 - _x1) / fw, (_y2 - _y1) / fh
        f.write("%d %f %f %f %f\n" % (_class_id, x, y, w, h))
    rospy.loginfo("add %s!" % s)


if __name__ == "__main__":
    rospy.init_node("coco_label_tool")
    rospy.loginfo("node start!")
    
    root = os.path.join(_DATASET_DIR, _DATASET_NAME)
    train_images = os.path.join(root, "train/images/")
    train_labels = os.path.join(root, "train/labels/")
    valid_images = os.path.join(root, "valid/images/")
    valid_labels = os.path.join(root, "valid/labels/")
    if not os.path.exists(root): os.makedirs(root)
    if not os.path.exists(train_images): os.makedirs(train_images)
    if not os.path.exists(train_labels): os.makedirs(train_labels)
    if not os.path.exists(valid_images): os.makedirs(valid_images)
    if not os.path.exists(valid_labels): os.makedirs(valid_labels)
    
    yaml = os.path.join(root, _DATASET_YAML)
    with open(yaml, "w") as f:
        f.write("path: %s\n" % root)
        f.write("train: train/images\n")
        f.write("val: valid/images\n")
        f.write("nc: %d\n" % len(_CLASSES))
        f.write("names: %s\n" % _CLASSES)
        
    code = os.path.join(root, "demo.py")
    with open(code, "w") as f:
        f.write("from ultralytics import YOLO\n")
        f.write("model = YOLO('%s.pt')\n" % _MODEL_BASE)
        f.write("results = model.train(\n")
        f.write("\tdata='%s',\n" % yaml)
        f.write("\timgsz=640,\n")
        f.write("\tepochs=%d,\n" % _EPOCHES)
        f.write("\tbatch=8,\n")
        f.write("\tname='yolov8n_custom'\n")
        f.write(")")
    
    _image = None
    topic_image = "/camera/rgb/image_raw"
    rospy.Subscriber(topic_image, Image, callback_image)
    rospy.wait_for_message(topic_image, Image)
    
    _class_id = 0
    _x1, _y1, _x2, _y2 = -1, -1, -1, -1
    _status = 0
    _train = True
    cv2.namedWindow("frame")  
    cv2.setMouseCallback("frame", callback_mouse)
    
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        
        frame = _image.copy()
        image = _image.copy()
        
        fh, fw, fc = frame.shape
        
        cv2.putText(frame, "%d(%s)" % (_class_id, "train" if _train else "valid"), (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3, cv2.LINE_AA)
        if _x1 != -1 and _y1 != -1 and _x2 != -1 and _y2 != -1:
            cv2.rectangle(frame, (_x1, _y1), (_x2, _y2), (0, 255, 0), 2)
        cv2.imshow("frame", frame)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break
        elif ord('0') <= key_code <= ord('9'):
            if key_code - ord('0') < len(_CLASSES):
                _class_id = key_code - ord('0')
        elif key_code in [32]:
            save()
    rospy.loginfo("node end!")
    
