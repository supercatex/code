import cv2
import time

print('camera')

size = 2
cam = [None, None, None]
for i in range(size):
    cam[i] = cv2.VideoCapture(i)

last_time = time.time()
while True:
    for i in range(size):
        ret, frame = cam[i].read()
        if not ret:
            print('Cannot read from your camera. index: ' + str(i))
            continue
            
        cv2.imshow('Camera ' + str(i), frame)
        if time.time() - last_time > 0.1:
            cv2.imwrite('frame' + str(i) + '.jpg', frame)
            last_time = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

for i in range(size):
    cam[i].release()
cv2.destroyAllWindows()
