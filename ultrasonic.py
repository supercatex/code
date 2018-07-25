# 載入需要用到的Module
import RPi.GPIO as GPIO
import time 

# 設置PIN位的命名方式 (BOARD, BCM)
GPIO.setmode(GPIO.BOARD)

# 定義需要使用的PIN位
TRIG = 11
ECHO = 12

# 初始化PIN位的功能 (IN, OUT)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

# 啟動Ultrasonic Sensor
GPIO.output(TRIG,0)
time.sleep(0.1)

GPIO.output(TRIG,1)
time.sleep(0.00001)

GPIO.output(TRIG,0)
# //啟動完成


# 記錄當ECHO PIN不為0時的時間
while GPIO.input(ECHO) == 0:
	pass
start = time.time()

# 記錄當ECHO PIN不為1時的時間
while GPIO.input(ECHO) == 1:
	pass
stop = time.time()

# 計算距離 2S=vt => v=340m/s, t=stop-start => S=170*t
S = 170 * (stop - start)
print (S, 'm')

# 重置GPIO所有PIN位
GPIO.cleanup() 
