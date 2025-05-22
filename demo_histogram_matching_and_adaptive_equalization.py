import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def load_image(path: str) -> np.array:
    img = cv2.imread(path)
    img = np.array(img, dtype=np.float32) / 255
    return img

def calc_hist(img: np.array) -> dict:
    colors = ["b", "g", "r"]
    hist = {}
    for i, c in enumerate(colors):
        hist[c] = cv2.calcHist([img], [i], None, [32], [0, 1])
    return hist

def color_matching(img: np.array) -> np.array:
    f_mean = []
    k = 0
    for i in range(3):
        f_mean.append(np.mean(img[:, :, i]))
        if f_mean[i] > f_mean[k]: k = i
    # print("B:", f_mean[0], "G:", f_mean[1], "R:", f_mean[2])

    new_img = img.copy()
    for i in range(3):
        new_img[:, :, i] = new_img[:, :, i] * (f_mean[k] / f_mean[i])
    new_img = np.clip(new_img, 0, 1)
    return new_img

def hsv_clahe(img: np.array) -> np.array:
    hsv = cv2.cvtColor(cm_img, cv2.COLOR_BGR2HSV)
    g = np.array(hsv[:, :, 2] * 255, dtype=np.uint8)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    g = clahe.apply(g)
    hsv[:, :, 2] = np.array(g, dtype=np.float32) / 255
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


for f_name in os.listdir("under_water_images"):
    or_img = load_image("under_water_images/%s" % f_name)
    or_hist = calc_hist(or_img)
    h, w, c = or_img.shape

    cm_img = color_matching(or_img)
    cm_hist = calc_hist(cm_img)

    eq_img = hsv_clahe(cm_img)
    eq_hist = calc_hist(eq_img)

    fig, axs = plt.subplots(3, 2, figsize=(12, 7))

    axs[0][0].imshow(cv2.cvtColor(or_img, cv2.COLOR_BGR2RGB))
    # axs[0][1].set_ylim(0, h * w // 3)
    for k, v in or_hist.items():
        axs[0][1].plot(v, c=k)

    axs[1][0].imshow(cv2.cvtColor(cm_img, cv2.COLOR_BGR2RGB))
    # axs[1][1].set_ylim(0, h * w // 3)
    for k, v in cm_hist.items():
        axs[1][1].plot(v, c=k)

    axs[2][0].imshow(cv2.cvtColor(eq_img, cv2.COLOR_BGR2RGB))
    # axs[2][1].set_ylim(0, h * w // 3)
    for k, v in eq_hist.items():
        axs[2][1].plot(v, c=k)

    thismanager = plt.get_current_fig_manager()
    thismanager.window.wm_geometry("+500+0")
    plt.show()
