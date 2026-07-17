import rclpy # rospy
from rclpy.node import Node
from bodyctrl_msgs.msg import MotorStatusMsg, \
    CmdSetMotorPosition, SetMotorPosition, \
    CmdSetMotorSpeed, SetMotorSpeed
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import numpy as np 
from rclpy.qos import QoSProfile, \
    QoSReliabilityPolicy, QoSDurabilityPolicy, \
    qos_profile_sensor_data


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

class WaistStatus(object):
    # -160 ~ +180 [31] Waist Yaw
    # - 45 ~ +120 [32] Waist Pitch
    def __init__(self, msg=None):
        self.msg = msg 
        self.motors: list[MotorStatus] = []
        for i in range(2): self.motors.append(MotorStatus())
        self.update(msg)
    
    def update(self, msg):
        if msg is None: return 
        for i in range(2):
            self.motors[i].update(msg.status[i])
    
    def __str__(self):
        return f"{self.motors[0]}{self.motors[1]}"

class LegStatus(object):
    # -13 ~ + 80 [51] Hip Pitch
    # -26 ~ +160 [52] Knee Pitch
    def __init__(self, msg=None):
        self.msg = msg 
        self.motors: list[MotorStatus] = []
        for i in range(2): self.motors.append(MotorStatus())
        self.update(msg)
    
    def update(self, msg):
        if msg is None: return 
        for i in range(2):
            self.motors[i].update(msg.status[i])

    def __str__(self):
        return f"{self.motors[0]}{self.motors[1]}"


class X_Humanoid(object):
    def __init__(self, node: Node):
        self.node = node 

        self.head_status = HeadStatus() 
        self.node.create_subscription(
            MotorStatusMsg, "/head/status",
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
        self.waist_cmd_pos = self.node.create_publisher(
            CmdSetMotorPosition,
            "/waist/cmd_pos", 10
        )
        self.leg_cmd_pos = self.node.create_publisher(
            CmdSetMotorPosition,
            "/leg/cmd_pos", 10
        )

        self.cb = CvBridge()
        self.head_image = None 
        self.head_depth = None
        self.node.create_subscription(
            Image, "/ob_camera_head/color/image_raw",
            self.callback_head_image,
            qos_profile_sensor_data

        )
        self.node.create_subscription(
            Image, "/ob_camera_head/depth/image_raw",
            self.callback_head_depth,
            qos_profile_sensor_data
        )

        self.waist_status = WaistStatus()
        self.node.create_subscription(
            MotorStatusMsg, "/waist/status",
            self.callback_waist_status, 10
        )

        self.leg_status = LegStatus()
        self.node.create_subscription(
            MotorStatusMsg, "/leg/status",
            self.callback_leg_status, 10
        )

    def callback_head_status(self, msg):
        self.head_status.update(msg)

    def callback_head_image(self, msg):
        self.head_image = self.cb.imgmsg_to_cv2(msg, "bgr8")

    def callback_head_depth(self, msg):
        self.head_depth = self.cb.imgmsg_to_cv2(msg, "passthrough")

    def callback_waist_status(self, msg):
        self.waist_status.update(msg)

    def callback_leg_status(self, msg):
        self.leg_status.update(msg)

    def check_head(self):
        if self.head_image is None: self.node.get_logger().warn("head image is not found."); return False
        if self.head_depth is None: self.node.get_logger().warn("head depth is not found."); return False 
        return True

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

    def set_waist_pos(self, p1, p2):
        msg = CmdSetMotorPosition()
        msg.cmds.append(self.create_SetMotorPosition(31, p1, 0.1, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(32, p2, 0.1, 0.5))
        self.waist_cmd_pos.publish(msg)

    def set_leg_pos(self, p1, p2):
        msg = CmdSetMotorPosition()
        msg.cmds.append(self.create_SetMotorPosition(51, p1, 0.1, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(52, p2, 0.1, 0.5))
        self.leg_cmd_pos.publish(msg)

    def set_height(self, h):
        L = 240
        h = max(150, min(h, 450))
        a = np.deg2rad(60) - np.arccos(h / 2 / L)

        m31, m32 = [x.pos for x in self.waist_status.motors]
        m51, m52 = [x.pos for x in self.leg_status.motors]
        e = ((m32 - a) ** 2 + (m51 + a) ** 2 + (m52 - a) ** 2) ** 0.5
        if e < 0.01: return False 

        self.set_waist_pos(m31, a)
        self.set_leg_pos(-a, a)
        return True
    
    def get_height(self):
        L = 240
        a = np.deg2rad(60) - self.leg_status.motors[1].pos
        h = np.cos(a) * L * 2
        return h
    

class Main(Node):
    def __init__(self):
        super().__init__("demo")
        self.timer = self.create_timer(1 / 20, self.callback_timer)
        self.robot = X_Humanoid(self)

    def callback_timer(self):
        if not self.robot.check_head(): return 

        # print(self.robot.head_status)
        # print(self.robot.waist_status)
        # print(self.robot.leg_status)

        # self.robot.set_head_pos(0.0, 0.0, 0.0)
        # self.robot.set_waist_pos(0 * 3.14 / 180, 26 * 3.14 / 180)
        # self.robot.set_leg_pos(-26 * 3.14 / 180, 26 * 3.14 / 180)
        self.robot.set_height(240)
        print(self.robot.get_height())

        cv2.imshow("frame", self.robot.head_image)
        cv2.imshow("depth", self.robot.head_depth)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            raise SystemExit
    

if __name__ == "__main__":
    try:
        rclpy.init()
        node = Main()
        rclpy.spin(node)
    except Exception as e:
        print("END with", e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
