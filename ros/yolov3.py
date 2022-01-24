import cv2
import numpy as np
import time

# https://github.com/PINTO0309/OpenVINO-YoloV3
# https://docs.openvinotoolkit.org/latest/omz_models_public_yolo_v3_tf_yolo_v3_tf.html


class DetectBox(object):
    LABELS = (
        "person", "bicycle", "car", "motorbike", "aeroplane",
        "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird",
        "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack",
        "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon",
        "bowl", "banana", "apple", "sandwich", "orange",
        "broccoli", "carrot", "hot dog", "pizza", "donut",
        "cake", "chair", "sofa", "potted plant", "bed",
        "dining table", "toilet", "tv monitor", "laptop", "mouse",
        "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock",
        "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    )
    SIDE = 13
    ANCHORS = {
        13: ((116, 90), (156, 198), (373, 326)),
        26: ((30, 61), (62, 45), (59, 119)),
        52: ((10, 13), (16, 30), (33, 23))
    }
    COORDINATE = 4
    THRESHOLD = 0.7
    IMG_W, IMG_H = 416, 416

    def __init__(self, x, y, w, h, class_id, confidence, img_w, img_h):
        w_scale = img_w / self.IMG_W
        h_scale = img_h / self.IMG_H
        self.x_min = int((x - w / 2) * w_scale)
        self.y_min = int((y - h / 2) * h_scale)
        self.x_max = int((x + w / 2) * w_scale)
        self.y_max = int((y + h / 2) * h_scale)
        self.class_id = class_id
        self.confidence = confidence

    def area(self) -> float:
        return (self.y_max - self.y_min) * (self.x_max - self.x_min)

    def draw(self, image, color=(64, 128, 64), thick=2, f_color=(255, 255, 255), f_scale=0.5, f_thick=1):
        cv2.rectangle(
            image,
            (self.x_min, self.y_min),
            (self.x_max, self.y_max),
            color, thick
        )
        text = "%s %.2f" % (self.LABELS[self.class_id], self.confidence)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.5
        w, h = cv2.getTextSize(text, font, f_scale, f_thick)[0]
        cv2.rectangle(
            image,
            (self.x_min, self.y_min - h),
            (self.x_min + w, self.y_min),
            color, -1
        )
        cv2.putText(
            image, text, (self.x_min, self.y_min),
            font, scale, f_color, f_thick, cv2.LINE_AA
        )

    @classmethod
    def calc_intersection_over_union(cls, box1, box2) -> float:
        w_overlap_area = min(box1.x_max, box2.x_max) - max(box1.x_min, box2.x_min)
        h_overlap_area = min(box1.y_max, box2.y_max) - max(box1.y_min, box2.y_min)
        overlap_area = 0.0
        if w_overlap_area > 0.0 and h_overlap_area > 0.0:
            overlap_area = w_overlap_area * h_overlap_area
        union_area = box1.area() + box2.area() - overlap_area
        if union_area <= 0.0:
            return 0.0
        return overlap_area / union_area

    @classmethod
    def parse_output(cls, output: np.ndarray, img_w: int, img_h: int) -> []:
        ret = []
        for row in range(cls.SIDE):
            for col in range(cls.SIDE):
                for n in range(len(cls.ANCHORS[cls.SIDE])):
                    index = n * (len(cls.LABELS) + cls.COORDINATE + 1)
                    confidence = output[0][index + cls.COORDINATE][row][col]
                    if confidence < cls.THRESHOLD:
                        continue
                    max_prob = 0.0
                    max_bbox = None
                    for i in range(len(cls.LABELS)):
                        class_index = index + cls.COORDINATE + 1 + i
                        prob = output[0][class_index][row][col] * confidence
                        if prob < cls.THRESHOLD or prob < max_prob:
                            continue
                        x = (col + output[0][index + 0][row][col]) * cls.IMG_W / cls.SIDE
                        y = (row + output[0][index + 1][row][col]) * cls.IMG_H / cls.SIDE
                        w = np.exp(output[0][index + 2][row][col]) * cls.ANCHORS[cls.SIDE][n][0]
                        h = np.exp(output[0][index + 3][row][col]) * cls.ANCHORS[cls.SIDE][n][1]
                        max_prob = prob
                        max_bbox = DetectBox(x, y, w, h, i, prob, img_w, img_h)
                    if max_bbox is not None:
                        ret.append(max_bbox)

        for i in range(len(ret)):
            for j in range(i + 1, len(ret)):
                if cls.calc_intersection_over_union(ret[i], ret[j]) >= 0.4:
                    ret[j].confidence = 0.0

        for i in range(len(ret) - 1, -1, -1):
            if ret[i].confidence < 0.2:
                ret.pop(i)

        return ret


if __name__ == "__main__":
    DetectBox.SIDE = 13
    _model_xml = "../models/Yolo-v3/Yolo-v3-13.xml"# % DetectBox.SIDE
    _model_bin = "../models/Yolo-v3/Yolo-v3-13.bin"# % DetectBox.SIDE
    _net = cv2.dnn.readNetFromModelOptimizer(_model_xml, _model_bin)
    _net.setPreferableBackend(cv2.dnn.DNN_BACKEND_INFERENCE_ENGINE)
    _net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    # _model_xml = "../models/Yolo-v3/yolov3.cfg"
    # _model_bin = "../models/Yolo-v3/yolov3.weights"
    # _net = cv2.dnn.readNetFromDarknet(_model_xml, _model_bin)

    _img_w, _img_h = 640, 480
    _camera = cv2.VideoCapture(0)
    _camera.set(cv2.CAP_PROP_FRAME_WIDTH, _img_w)
    _camera.set(cv2.CAP_PROP_FRAME_HEIGHT, _img_h)
    while _camera.isOpened():
        _t1 = time.time()

        _success, _frame = _camera.read()
        if not _success:
            break

        _input_blob = cv2.dnn.blobFromImage(
            _frame,
            scalefactor=1.0,
            size=(DetectBox.IMG_W, DetectBox.IMG_H),
            mean=(0, 0, 0),
            swapRB=False,
            crop=False
        )
        _net.setInput(_input_blob)
        # _last_layer = _net.getUnconnectedOutLayersNames()
        _out = _net.forward()
        print(_out.shape)
        # _out = np.reshape(_out, (1, 255, 52, 52))
        _objects = DetectBox.parse_output(_out, _img_w, _img_h)
        print(len(_objects))
        for _obj in _objects:
            _obj.draw(_frame)

        cv2.imshow("frame", _frame)
        _key_code = cv2.waitKey(1)
        if _key_code in [27, ord('q')]:
            break
        print("fps: %.2f" % (1.0 / (time.time() - _t1)))
        del _frame, _input_blob, _out, _objects
    _camera.release()
    cv2.destroyAllWindows()
    del _net
