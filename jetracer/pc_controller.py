from pyPS4Controller.controller import Controller
from jetracer.nvidia_racecar import NvidiaRacecar
import time
import threading

car = NvidiaRacecar()
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.y = 0.0
        self.x = 0.0
        self.max_value = 32767
        self.ky = 0.12
        self.kx = 0.8
        self.bx = 0.0

    def on_x_press(self):
        self.x = 0.0
        self.y = 0.0
        self.run()

    def on_L3_up(self, value):
        self.y = value / self.max_value
        self.run()
 
    def on_L3_down(self, value):
        self.y = value / self.max_value
        x = self.x
        y = self.y
        car.steering = -(x * self.kx + self.bx)
        car.throttle = -y * 0.3
    
    def on_L3_x_at_rest(self):
        self.y = 0.0
        self.run()
        
    def on_L3_y_at_rest(self):
        self.y = 0.0
        self.run()

    def on_L2_press(self, value):
        self.x = min(max(-1, value / 28370), 1)
        #print(value, self.x)
        self.run()
    
    def run(self):
        car.steering = -(self.x * self.kx + self.bx)
        car.throttle = -self.y * self.ky
    
        

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)

def test():
    global controller
    controller.listen(timeout=60)

thread1 = threading.Thread(target=test)
thread1.setDaemon(False)
thread1.start()

while True:
    time.sleep(1)
    print(car.steering, car.throttle)
