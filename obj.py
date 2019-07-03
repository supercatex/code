# organize imports
import numpy as np
from keras.models import Model
from keras.preprocessing import image
from keras.applications import imagenet_utils, mobilenet
import cv2


def process_image(img_path):
    img = image.load_img(img_path, target_size=(224, 224))
    img_array = image.img_to_array(img)
    img_array = np.expand_dims(img_array, axis=0)
    pImg = mobilenet.preprocess_input(img_array)
    return pImg


if __name__ == '__main__':

    mbn = mobilenet.MobileNet()

    cap = cv2.VideoCapture(0)

    while True:
        _, img = cap.read()
        pimg = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        dt = int((pimg.shape[1] - pimg.shape[0]) / 2)
        pimg = pimg[:, dt:pimg.shape[1]-dt]
        print(dt, pimg.shape)
        pimg = cv2.resize(pimg, (224, 224))
        cv2.imshow("image", img)
        cv2.imshow("pimg", pimg)
        pimg = np.expand_dims(pimg, axis=0)
        pimg = mobilenet.preprocess_input(pimg)

        # test_img_path = "test.jpg"
        # cv2.imwrite(test_img_path, img)
        # pImg = process_image(test_img_path)

        if cv2.waitKey(1) == 27:
            break

        prediction = mbn.predict(pimg)
        results = imagenet_utils.decode_predictions(prediction)
        print(results)

    cap.release()
