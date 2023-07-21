from jetcam.csi_camera import CSICamera
import ipywidgets
from IPython.display import display
import traitlets
from jetcam.utils import bgr8_to_jpeg

camera = CSICamera(width=224, height=224)
camera.running = True
camera.unobserve_all()
camera_widget = ipywidgets.Image(width=camera.width, height=camera.height)
traitlets.dlink((camera, 'value'), (camera_widget, 'value'), transform=bgr8_to_jpeg)

display(camera_widget)

# Part 2
import time
import cv2
import os
import shutil

from jetracer.nvidia_racecar import NvidiaRacecar
car = NvidiaRacecar()

c = 1
t_time = 0.5
folder = "./my_data/apex/"
if os.path.exists(folder):
    shutil.rmtree(folder)
os.makedirs(folder)

sw = True
b_time = time.time()
while True:
    print(car.steering)

    if time.time() - b_time < t_time:
        continue
    
    if sw:
        snapshot = camera.value.copy()
        cv2.imwrite(os.path.join(folder, "%08d.jpg" % (c)), snapshot)
        c += 1
    b_time = time.time()
