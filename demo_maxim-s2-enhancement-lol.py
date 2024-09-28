from huggingface_hub import from_pretrained_keras
from PIL import Image

import tensorflow as tf
import numpy as np
import requests
import cv2

model = from_pretrained_keras("google/maxim-s2-enhancement-lol")

url = "https://github.com/sayakpaul/maxim-tf/raw/main/images/Enhancement/input/748.png"
image = Image.open(requests.get(url, stream=True).raw)
image = np.array(image)
h, w, c = image.shape

k = 0
while True:
    org = image.copy()
    image = tf.convert_to_tensor(image)
    image = tf.image.resize(image, (256, 256))

    predictions = model.predict(tf.expand_dims(image, 0))
    print(predictions)
    print(len(predictions))
    print(len(predictions[0]), len(predictions[1]))

    # im = img[0] / 255
    img = predictions[1][2]
    im = np.array(img[0], np.float32) / 255
    im = cv2.resize(im, (w, h))
    im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    cv2.imshow("img" + str(k), im)
    cv2.waitKey(0)

    k += 1
    image = cv2.cvtColor(im, cv2.COLOR_BGR2RGB) * 255

