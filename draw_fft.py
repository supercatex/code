import cv2
import numpy as np

def transform(data, cx, cy):
    y = [complex(-p[1] + cy, -p[0] + cx) for p in data]
    yy = np.fft.fft(y)

    ly = len(y)
    pp = []
    for i, v in enumerate(yy[:len(y)]):
        c = -2 * np.pi * i / ly
        pp.append([-v.real / ly, c, np.pi / 2])
        pp.append([-v.imag / ly, c, 0])
    pp.sort(key=lambda x: abs(x[0]), reverse=True)
    return pp


h, w, c = 400, 400, 3
draw = np.zeros((h, w, c), dtype=np.uint8)
p = []
data = []

kkk = 0
def mouse_event_handler(event, x, y, flags, param):
    global data, kkk
    if event == cv2.EVENT_MOUSEMOVE and kkk % 20 == 0:
        data.append([x, y])
        cv2.circle(draw, (int(x), int(y)), 1, (255, 255, 255), -1)
        cv2.imshow("draw", draw)
    kkk += 1
cv2.imshow("draw", draw)
cv2.waitKey(0)
cv2.setMouseCallback("draw", mouse_event_handler)
cv2.waitKey(0)

for f in transform(data, w / 2, h / 2):
    p.append([f[2], f[0], f[1]])

px, py = -1, -1
bak = np.zeros((h, w, c), dtype=np.uint8)
while True:
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cx, cy = w // 2, h // 2
    cv2.circle(img, (cx, cy), 1, (0, 0, 255), -1)
    for i, (a, r, ax) in enumerate(p):
        ra = a
        x = cx + np.cos(ra) * r
        y = cy + np.sin(ra) * r
        cv2.circle(img, (int(x), int(y)), 3, (255, 0, 0), -1)
        p[i][0] = p[i][0] + ax
        # if p[i][0] >= 2 * np.pi: p[i][0] -= 2 * np.pi
        # if p[i][0] <= -2 * np.pi: p[i][0] += 2 * np.pi
        cx, cy = x, y
    # cv2.circle(bak, (int(cx), int(cy)), 1, (255, 255, 255), -1)
    if px != -1: cv2.line(bak, (int(px), int(py)), (int(cx), int(cy)), (255, 255, 255), 1)
    px, py = cx, cy

    cv2.imshow("img", cv2.bitwise_or(img, bak, img))
    key_code = cv2.waitKey(10)
    if key_code in [27, ord('q')]:
        break
