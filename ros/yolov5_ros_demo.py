#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
import os
import torch
from torchvision.models.detection.mask_rcnn import maskrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import fasterrcnn_mobilenet_v3_large_320_fpn

COCO_CLASSES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

def callback_image(msg):
    global _frame
    _frame = np.frombuffer(
                msg.data, dtype=np.uint8
            ).reshape(msg.height, msg.width, -1)[:, :, ::-1]

def callback_depth(msg):
    global _depth
    _depth = np.frombuffer(
                msg.data, dtype=np.uint16
            ).reshape(msg.height, msg.width)

if __name__ == "__main__":
    rospy.init_node("my_pkg_demo")
    rospy.loginfo("my_pkg_demo start!")

    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)

    os.environ["TORCH_HOME"] = "~"
    net = torch.hub.load("ultralytics/yolov5", "yolov5l", device="cpu") # yolov5
    # net = maskrcnn_resnet50_fpn(pretrained=True).eval() # maskrcnn
    # net = fasterrcnn_mobilenet_v3_large_320_fpn(pretrained=True).eval() # fasterrcnn

    _frame = None
    _depth = None
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        if _frame is None: continue
        if _depth is None: continue

        frame = _frame.copy()
        depth = _depth.copy()

        ''' # fasterrcnn
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = torch.as_tensor(img, dtype=torch.float32)
        img = img / 255
        img = torch.permute(img, (2, 0, 1))
        img = torch.unsqueeze(img, dim=0)
        res = net(img)[0]
        boxes = res["boxes"]
        labels = res["labels"]
        scores = res["scores"]
        for i in range(len(boxes)):
            if scores[i] < 0.7: continue
            x1, y1, x2, y2 = map(int, boxes[i])
            color = (0, 255, 0)
            if COCO_CLASSES[labels[i]] == "bottle":
                color = (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame, COCO_CLASSES[labels[i]],
                (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, color, 2
            )
        #'''

        ''' # maskrcnn
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = torch.as_tensor(img, dtype=torch.float32)
        img = img / 255
        img = torch.permute(img, (2, 0, 1))
        img = torch.unsqueeze(img, dim=0)
        res = net(img)[0]
        boxes = res["boxes"]
        labels = res["labels"]
        scores = res["scores"]
        masks = res["masks"].detach().numpy()
        for i in range(len(boxes)):
            if scores[i] < 0.7: continue
            mask = masks[i][0]
            mask = np.array(mask * 255, dtype=np.uint8)
            mask = cv2.inRange(mask, 0, 127)
            indices = np.where(mask == 0)
            frame[indices[0], indices[1], 0] = 200
        #'''

        #''' # yolov5
        res = net(frame)
        for x1, y1, x2, y2, pred, index in res.xyxy[0]:
            if pred < 0.7: continue
            x1, y1, x2, y2, index = map(int, (x1, y1, x2, y2, index))
            name = net.names[index]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame, name, (x1, y1 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, 
                (0, 255, 0), 1, cv2.LINE_AA
            )
        #'''

        cv2.imshow("frame", frame)
        cv2.imshow("depth", depth)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break

    rospy.loginfo("my_pkg_demo end.")
