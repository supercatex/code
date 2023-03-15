import cv2
import time

cap1 = cv2.VideoCapture("my_robot_1.mp4")
cap2 = cv2.VideoCapture("my_robot_2.mp4")
n_frame = cap2.get(cv2.CAP_PROP_FRAME_COUNT)
c_frame = 0
fps = 120
speaking = False
while True:
    s_time = time.time()

    if not speaking:
        success, frame = cap1.read()
    else:
        success, frame = cap2.read()
    if not success: break

    c_frame += 1
    if c_frame == n_frame:
        c_frame = 0
        cap1.set(cv2.CAP_PROP_POS_FRAMES, 0)
        cap2.set(cv2.CAP_PROP_POS_FRAMES, 0)

    cv2.imshow("frame", frame)

    key_code = -1
    while time.time() - s_time < 1 / fps:
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            key_code = -2
            break
        if key_code == 32:
            speaking = not speaking
            cap1.set(cv2.CAP_PROP_POS_FRAMES, 0)
            cap2.set(cv2.CAP_PROP_POS_FRAMES, 0)
    if key_code == -2: break
cap1.release()
cap2.release()
cv2.destroyAllWindows()
