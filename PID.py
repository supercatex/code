import time


class PID():
    
    Kp = 0
    Ki = 0
    Kd = 0
    dt = 1
    limit = 0


    def __init__(self, P = 0, I = 0, D = 0, dt = 1, limit = 0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.dt = dt
        self.limit = limit
        self.clear()


    def clear(self):
        self.__last_error = 0
        self.__integral_error = 0
        self.__last_time = time.time()


    def update(self, error):
        current_time = time.time()
        delta_time = current_time - self.__last_time
        if delta_time == 0: return 0
        dt = self.dt / delta_time

        self.__integral_error += error * dt
        delta_error = (error - self.__last_error) / dt

        output = self.Kp * error + self.Ki * self.__integral_error + self.Kd * delta_error
        if self.limit != 0:
            output = max(min(output, self.limit), -self.limit) / self.limit

        self.__last_error = error
        self.__last_time = current_time
        return output


    def __str__(self):
        return 'PID (P = ' + str(self.Kp) + ', I = ' + str(self.Ki) + ', D = ' + str(self.Kd) + ')'


if __name__ == '__main__':

    current_temp = 0
    target_temp = 40

    controller = PID(8, 0.3, 1, 1, 10)
    print(controller)
    
    max_value = -9999
    min_value = 99999
    while True:
        time.sleep(1)
        error = target_temp - current_temp
        output = controller.update(error)
        current_temp += output * 7
        if max_value < current_temp: max_value = current_temp
        if min_value > current_temp: min_value = current_temp
        current_temp *= 0.9
        print('Power: ' + str(round(output, 2)) + \
            '\tTemp: ' + str(round(current_temp, 2)) + \
            '\tMAX: ' + str(round(max_value, 2)) + '\tMIN: ' + str(round(min_value, 2)))