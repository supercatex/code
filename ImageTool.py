import cv2 as cv
import time
import random


class ImageTool(object):

    def __init__(self, debug=True):
        self.video = cv.VideoCapture(0)
        self.frame = None

        self.pause = False
        self.mode = 0
        self.source_frame = None

        self.ref_scale_rate = 1
        self.ref_pos_start = (0, 0)
        self.ref_pos_end = (0, 0)

        self.tmp_pos_start = []
        self.tmp_pos_end = []

        self.detect_color_from = (0, 0, 0)
        self.detect_color_to = (180, 255, 255)
        self.e1 = 10
        self.e2 = 150
        self.e3 = 100

    def capture(self):
        success, self.frame = self.video.read()
        return self.frame

    def on_mouse_frame(self, event, x, y, flags, param):
        if self.pause:
            if event == cv.EVENT_RBUTTONDOWN:
                self.mode = 1
                self.ref_pos_start = (x, y)
                self.tmp_pos_start = []
                self.tmp_pos_end = []

            if event == cv.EVENT_RBUTTONUP:
                self.mode = 0

            if event == cv.EVENT_LBUTTONDOWN:
                self.mode = 2
                self.tmp_pos_start.append((x, y))

            if event == cv.EVENT_LBUTTONUP:
                self.mode = 0
                self.tmp_pos_end.append((x, y))

            if event == cv.EVENT_MOUSEMOVE:
                if self.mode == 1:
                    image = self.source_frame.copy()
                    self.ref_pos_end = (x, y)
                    cv.line(image, self.ref_pos_start, self.ref_pos_end, (0, 255, 255), 2, cv.LINE_AA)
                    self.frame = image

                if self.mode == 2:
                    image = self.source_frame.copy()
                    cv.line(image, self.ref_pos_start, self.ref_pos_end, (0, 255, 255), 2, cv.LINE_AA)
                    cv.line(image, self.tmp_pos_start[-1], (x, y), (0, 255, 0), 2, cv.LINE_AA)
                    for i in range(len(self.tmp_pos_end)):
                        cv.line(image, self.tmp_pos_start[i], self.tmp_pos_end[i], (0, 255, 0), 2, cv.LINE_AA)
                    self.frame = image
        else:
            if event == cv.EVENT_LBUTTONDBLCLK:
                temp = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)
                h, s, v = temp[y, x]
                h = int(h)
                s = int(s)
                v = int(v)
                self.detect_color_from = (max(h - self.e1, 0), max(s - self.e2, 16), max(v - self.e3, 16))
                self.detect_color_to = (min(h + self.e1, 180), min(s + self.e2, 240), min(v + self.e3, 240))
            if event == cv.EVENT_RBUTTONDBLCLK:
                self.detect_color_from = (0, 0, 0)
                self.detect_color_to = (180, 255, 255)


if __name__ == "__main__":
    tool = ImageTool()
    cv.namedWindow("frame")
    cv.setMouseCallback("frame", tool.on_mouse_frame)

    while True:
        if not tool.pause:
            frame = tool.capture()
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, tool.detect_color_from, tool.detect_color_to)
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            cv.drawContours(frame, contours, -1, (0, 255, 0), 2)
        else:
            frame = tool.frame

        cv.imshow("frame", frame)

        key = cv.waitKey(1)
        if key == 27:
            break
        if key == 32:
            tool.pause = not tool.pause
            frame = tool.capture()
            tool.source_frame = frame.copy()
        if key == ord('s'):
            file = "output/IMG_%s_%d.jpg" % (time.strftime("%Y%m%d_%H%M%S", time.localtime()), random.randint(1, 1000))
            cv.imwrite(file, tool.frame)

    tool.video.release()
    cv.destroyAllWindows()
