import rclpy
from rclpy.node import Node
from bodyctrl_msgs.msg import MotorStatusMsg, \
    CmdSetMotorPosition, SetMotorPosition, \
    CmdSetMotorSpeed, SetMotorSpeed


class MotorStatus(object):
    # error code
    # 33072     設備掉線
    # 33073     關節超限
    # 1         電機過熱
    # 2         過流
    # 3         電壓過低
    # 4         MOS過熱
    # 5         堵轉
    # 6         電壓過高
    # 7         缺相
    # 8         編碼器錯誤
    def __init__(self, msg=None):
        self.msg = msg          # RAW data
        self.name = ""          # Motor Name
        self.pos = 0.0          # rad
        self.speed = 0.0        # rad
        self.current = 0.0      # A
        self.temperature = 0.0  # Mos
        self.error = 0
        self.update(msg)

    def update(self, msg):
        if msg is None: return
        self.name = msg.name 
        self.pos = msg.pos 
        self.speed = msg.speed 
        self.current = msg.current
        self.temperature = msg.temperature
        self.error = msg.error 

    def __str__(self):
        return f'''MotorName: {self.name}
    position: {self.pos}
    speed: {self.speed}
    current: {self.current}
    temperature: {self.temperature}
    error: {self.error}
'''

class RPY_Position(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def __str__(self):
        return f"ROLL: {self.x:.2f}, PITCH: {self.y:.2f}, YAW: {self.z:.2f}"

class HeadStatus(object):
    # Kp: 0~2000
    # Kd: 0~300
    # -26 ~ +26 [1] Roll
    # -25 ~ +25 [2] Pitch
    # -90 ~ +90 [3] Yaw
    def __init__(self, msg=None):
        self.msg = msg
        self.motors: list[MotorStatus] = []
        for i in range(3): self.motors.append(MotorStatus())
        self.pos = RPY_Position() 
        self.update(msg)

    def update(self, msg):
        if msg is None: return
        for i in range(3):
            self.motors[i].update(msg.status[i])
        self.pos.x = msg.status[0].pos
        self.pos.y = msg.status[1].pos 
        self.pos.z = msg.status[2].pos 

    def __str__(self):
        return f"{self.pos}\n{self.motors[0]}{self.motors[1]}{self.motors[2]}"

class X_Humanoid(object):
    def __init__(self, node):
        self.node = node 

        self.head_status = HeadStatus() 
        self.node.create_subscription(
            MotorStatusMsg,
            "/head/status",
            self.callback_head_status, 10
        )
        self.head_cmd_pos = self.node.create_publisher(
            CmdSetMotorPosition,
            "/head/cmd_pos", 10
        )
        self.head_cmd_vel = self.node.create_publisher(
            CmdSetMotorSpeed,
            "/head/cmd_vel", 10
        )

    def callback_head_status(self, msg):
        self.head_status.update(msg) 

    def create_SetMotorPosition(self, name, pos, spd, cur):
        msg = SetMotorPosition()
        msg.name = name 
        msg.pos = pos 
        msg.spd = spd 
        msg.cur = cur 
        return msg 
    
    def create_SetMotorSpeed(self, name, spd, cur):
        msg = SetMotorSpeed()
        msg.name = name  
        msg.spd = spd 
        msg.cur = cur 
        return msg 

    def set_head_pos(self, x, y, z):
        msg = CmdSetMotorPosition()
        msg.cmds.append(self.create_SetMotorPosition(1, x, 0.5, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(2, y, 0.5, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(3, z, 0.5, 0.5))
        self.head_cmd_pos.publish(msg)

    def set_head_vel(self, x, y, z):
        msg = CmdSetMotorSpeed()
        msg.cmds.append(self.create_SetMotorSpeed(1, x, 0.5))
        msg.cmds.append(self.create_SetMotorSpeed(2, y, 0.5))
        msg.cmds.append(self.create_SetMotorSpeed(3, z, 0.5))
        self.head_cmd_vel.publish(msg)


class Main(Node):
    def __init__(self):
        super().__init__("demo")
        self.timer = self.create_timer(1 / 20, self.callback_timer)
        self.robot = X_Humanoid(self)
        self.i = 0

    def callback_timer(self):
        if self.robot.head_status is not None:
            obj = self.robot.head_status
            print(obj)
            self.i += 1
            if 80 <= self.i <= 100:
                self.robot.set_head_vel(0.0, 0.3, 0.0)

            

if __name__ == "__main__":
    rclpy.init()
    node = Main()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
