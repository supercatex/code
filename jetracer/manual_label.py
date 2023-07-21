import cv2
import os
from pathlib import Path


def callback_click(event, x, y, flags, param):
    global _x, _y, _img, img
    if event == cv2.EVENT_LBUTTONUP:
        _x = x
        _y = y
        _img = img.copy()
        cv2.circle(_img, (_x, _y), 3, (0, 255, 0), -1)
        cv2.imshow("frame", _img)


cv2.namedWindow("frame")
cv2.setMouseCallback("frame", callback_click)

folder = "D:\\datasets\\apex\\"
for i, f in enumerate(sorted(Path(folder).iterdir(), key=os.path.getmtime)):
    f = str(f).replace(folder, "")
    print(i, f, os.path. join(folder, f))
    if f[0] == '.': continue

    is_new = True
    _x, _y = 224 // 2, 224 // 2
    if len(f.split("_")) == 3:
        is_new = False
        _x = int(f.split("_")[0])
        _y = int(f.split("_")[1])

    img = cv2.imread(os.path.join(folder, f))
    _img = img.copy()
    cv2.circle(_img, (_x, _y), 3, (0, 255, 0), -1)
    cv2.imshow("frame", _img)
    key_code = cv2.waitKey(0)
    cv2.imwrite(os.path.join(folder, "%d_%d_%s" % (_x, _y, f)), img)
    os.remove(os.path.join(folder, f))
