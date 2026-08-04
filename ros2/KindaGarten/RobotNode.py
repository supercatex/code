from KindaGarten.basic import * 
from bodyctrl_msgs.srv import SetAngleFlexible, SetForce, SetGestureForceCalibration
from xarm_sdk import XARM_manager, TopicPublisher, ParamConfiger
from xarm_sdk.utils import *
from bodyctrl_msgs.msg import MotorStatusMsg, \
    CmdSetMotorPosition, SetMotorPosition, \
    CmdSetMotorSpeed, SetMotorSpeed
from geometry_msgs.msg import Pose
from lyre_msgs.srv import PlayText, PlayFile, PlayStop


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

class JointState(object):
    def __init__(self, msg=None):
        self.msg = msg          # RAW data
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.update(msg)

    def update(self, msg):
        if msg is None: return
        self.name = msg.name 
        self.position = msg.position 
        self.velocity = msg.velocity
        self.effort = msg.effort

    def __str__(self):
        return f'''JointState:
    name: {self.name}
    position: {self.position}
    velocity: {self.velocity}
    effort: {self.effort}
'''

class RPY_Position(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def __str__(self):
        return f"ROLL: {self.x:.2f}, PITCH: {self.y:.2f}, YAW: {self.z:.2f}"

    def tolist(self):
        return [self.x, self.y, self.z]

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
        self.motors_pre: list[MotorStatus] = []
        for i in range(3): self.motors_pre.append(MotorStatus())
        self.rpy = RPY_Position() 
        self.update(msg)

    def update(self, msg):
        if msg is None: return
        for i in range(3):
            self.motors_pre[i].update(self.motors[i])
        for i in range(3):
            self.motors[i].update(msg.status[i])
        self.rpy.x = msg.status[0].pos
        self.rpy.y = msg.status[1].pos 
        self.rpy.z = msg.status[2].pos 

    def __str__(self):
        return f"{self.rpy}\n{self.motors[0]}{self.motors[1]}{self.motors[2]}"

class WaistStatus(object):
    # -160 ~ +180 [31] Waist Yaw
    # - 45 ~ +120 [32] Waist Pitch
    def __init__(self, msg=None):
        self.msg = msg 
        self.motors: list[MotorStatus] = []
        for i in range(2): self.motors.append(MotorStatus())
        self.motors_pre: list[MotorStatus] = []
        for i in range(2): self.motors_pre.append(MotorStatus())
        self.update(msg)
    
    def update(self, msg):
        if msg is None: return 
        for i in range(2):
            self.motors_pre[i].update(self.motors[i])
        for i in range(2):
            self.motors[i].update(msg.status[i])
    
    def __str__(self):
        return "".join([str(x) for x in self.motors])

class LegStatus(object):
    # -13 ~ + 80 [51] Hip Pitch
    # -26 ~ +160 [52] Knee Pitch
    def __init__(self, msg=None):
        self.msg = msg 
        self.motors: list[MotorStatus] = []
        for i in range(2): self.motors.append(MotorStatus())
        self.motors_pre: list[MotorStatus] = []
        for i in range(2): self.motors_pre.append(MotorStatus())
        self.update(msg)
    
    def update(self, msg):
        if msg is None: return 
        for i in range(2):
            self.motors_pre[i].update(self.motors[i])
        for i in range(2):
            self.motors[i].update(msg.status[i])

    def __str__(self):
        return "".join([str(x) for x in self.motors])

class HandStatus(object):
    def __init__(self, msg=None):
        self.msg = msg 
        self.motors: list[MotorStatus] = []
        for i in range(6): self.motors.append(MotorStatus())
        self.motors_pre: list[MotorStatus] = []
        for i in range(6): self.motors_pre.append(MotorStatus())
        self.update(msg)
    
    def update(self, msg):
        if msg is None: return 
        for i in range(6):
            self.motors_pre[i].update(self.motors[i])
        for i in range(6):
            self.motors[i].update(msg.status[i])

    def __str__(self):
        return "".join([str(x) for x in self.motors])

class ArmStatus(object):
    def __init__(self, msg=None):
        self.msg = msg 
        self.motors: list[MotorStatus] = []
        for i in range(14): self.motors.append(MotorStatus())
        self.motors_pre: list[MotorStatus] = []
        for i in range(14): self.motors_pre.append(MotorStatus())
        self.update(msg)
    
    def update(self, msg):
        if msg is None: return 
        for i in range(14):
            self.motors_pre[i].update(self.motors[i])
        for i in range(14):
            self.motors[i].update(msg.status[i])

    def __str__(self):
        return "".join([str(x) for x in self.motors])


class XarmHandler(object):
    def __init__(self):
        self.xarm = XARM_manager()
        self.deactivate()
        self.topic_publisher = TopicPublisher(self.xarm)

        value = 0.0005
        param_configer = ParamConfiger(self.xarm)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "otg_p_step", value)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "otg_r_step", value)

        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "otg_p_step", value)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "otg_r_step", value)

        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "otg_p_step", value)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "otg_r_step", value)

        self.xarm.hardware_arm_enable(True)

    def deactivate(self):
        self.xarm.xarm_deactivate_all_controller()
        self.xarm.get_logger().info("deactivate all controller.")

    def set_endpose_L(self, xyz, rpy, deg=False, delay=3.0):
        x, y, z = xyz 
        if deg: rpy = np.deg2rad(rpy)

        msg_pose = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose.position.x = x
        msg_pose.position.y = y
        msg_pose.position.z = z

        q = euler_to_quaternion(*rpy)
        msg_pose.orientation.x = q[0]
        msg_pose.orientation.y = q[1]
        msg_pose.orientation.z = q[2]
        msg_pose.orientation.w = q[3]

        self.xarm.xarm_deactivate_all_controller()
        self.xarm.activate_controller("endpose_single_arm_qp_L_controller")
        self.topic_publisher.publish_endposetarget_L(msg_pose, from_frame="waist_yaw_link")
        time.sleep(delay)
        self.xarm.xarm_deactivate_all_controller()
    
    def set_endpose_R(self, xyz, rpy, deg=False, delay=3.0):
        x, y, z = xyz 
        if deg: rpy = np.deg2rad(rpy)

        msg_pose = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose.position.x = x
        msg_pose.position.y = y
        msg_pose.position.z = z

        q = euler_to_quaternion(*rpy)
        msg_pose.orientation.x = q[0]
        msg_pose.orientation.y = q[1]
        msg_pose.orientation.z = q[2]
        msg_pose.orientation.w = q[3]

        self.xarm.xarm_deactivate_all_controller()
        self.xarm.activate_controller("endpose_single_arm_qp_R_controller")
        self.topic_publisher.publish_endposetarget_R(msg_pose, from_frame="waist_yaw_link")
        time.sleep(delay)
        self.xarm.xarm_deactivate_all_controller()

    def set_endpose_LR(self, xyz_L, rpy_L, xyz_R, rpy_R, deg=False, delay=3.0):
        x1, y1, z1 = xyz_L
        if deg: rpy_L = np.deg2rad(rpy_L)
        x2, y2, z2 = xyz_R 
        if deg: rpy_R = np.deg2rad(rpy_R)

        msg_pose1 = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose1.position.x = x1
        msg_pose1.position.y = y1
        msg_pose1.position.z = z1

        q = euler_to_quaternion(*rpy_L)
        msg_pose1.orientation.x = q[0]
        msg_pose1.orientation.y = q[1]
        msg_pose1.orientation.z = q[2]
        msg_pose1.orientation.w = q[3]

        msg_pose2 = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose2.position.x = x2
        msg_pose2.position.y = y2
        msg_pose2.position.z = z2

        q = euler_to_quaternion(*rpy_R)
        msg_pose2.orientation.x = q[0]
        msg_pose2.orientation.y = q[1]
        msg_pose2.orientation.z = q[2]
        msg_pose2.orientation.w = q[3]

        self.xarm.xarm_deactivate_all_controller()
        self.xarm.activate_controller("endpose_dual_arm_qp_controller")
        self.topic_publisher.publish_endposetarget_dual(msg_pose1, msg_pose2, from_frame_l="waist_yaw_link", from_frame_r="waist_yaw_link")
        time.sleep(delay)
        self.xarm.xarm_deactivate_all_controller()

    def set_arm_joints(self, pos_L, pos_R, delay=5.0):
        self.xarm.xarm_deactivate_all_controller()
        self.xarm.activate_controller("jointspace_arm_L_controller")
        self.xarm.activate_controller("jointspace_arm_R_controller")
        self.topic_publisher.publish_jointspace_commands_L(pos_L)
        self.topic_publisher.publish_jointspace_commands_R(pos_R)
        time.sleep(delay)
        self.xarm.xarm_deactivate_all_controller()


class RobotNode(Node):
    def __init__(self):
        super().__init__("RobotNode")
        self.head_status = HeadStatus() 
        self.waist_status = WaistStatus()
        self.leg_status = LegStatus()
        SubscriptionNode(MotorStatusMsg, "/head/status", lambda x: self.head_status.update(x))
        SubscriptionNode(MotorStatusMsg, "/waist/status", lambda x: self.waist_status.update(x))
        SubscriptionNode(MotorStatusMsg, "/leg/status", lambda x: self.leg_status.update(x))
        self.arm_status = ArmStatus()
        SubscriptionNode(MotorStatusMsg, "/arm/status", lambda x: self.arm_status.update(x))
        
        self.head_cmd_pos = self.create_publisher(
            CmdSetMotorPosition,
            "/head/cmd_pos", 10
        )
        self.waist_cmd_pos = self.create_publisher(
            CmdSetMotorPosition,
            "/waist/cmd_pos", 10
        )
        self.leg_cmd_pos = self.create_publisher(
            CmdSetMotorPosition,
            "/leg/cmd_pos", 10
        )

        self.set_angle_left_hand = self.create_client(SetAngleFlexible, "/inspire_hand/set_angle_flexible/left_hand")
        self.set_angle_right_hand = self.create_client(SetAngleFlexible, "/inspire_hand/set_angle_flexible/right_hand")
        self.set_force_left_hand = self.create_client(SetForce, "/inspire_hand/set_force/left_hand")
        self.set_force_right_hand = self.create_client(SetForce, "/inspire_hand/set_force/right_hand")
        self.set_gesture_force_calibration_L = self.create_client(SetGestureForceCalibration, "/inspire_hand/set_gesture_force_calibration/left_hand")
        self.set_gesture_force_calibration_R = self.create_client(SetGestureForceCalibration, "/inspire_hand/set_gesture_force_calibration/right_hand")
        self.play_text = self.create_client(PlayText, "/audio_play/play_text")
        self.play_file = self.create_client(PlayFile, "/audio_play/play_file")
        self.play_stop = self.create_client(PlayStop, "/audio_play/stop")
        self.arm = XarmHandler()

        self.ec = 100

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

    def set_head_pos(self, rpy, deg=False, delay=3.0):
        if deg: rpy = np.deg2rad(rpy)
        x, y, z = rpy

        msg = CmdSetMotorPosition()
        msg.cmds.append(self.create_SetMotorPosition(1, x, 1.0, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(2, y, 1.0, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(3, z, 1.0, 0.5))
        self.head_cmd_pos.publish(msg)
        time.sleep(delay)

    def set_waist_pos(self, p1, p2, deg=False, delay=5.0):
        if deg: 
            p1 = np.deg2rad(p1)
            p2 = np.deg2rad(p2)
        msg = CmdSetMotorPosition()
        msg.cmds.append(self.create_SetMotorPosition(31, p1, 0.3, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(32, p2, 0.3, 0.5))
        self.waist_cmd_pos.publish(msg)
        time.sleep(delay)

    def set_leg_pos(self, p1, p2, deg=False, delay=5.0):
        if deg: 
            p1 = np.deg2rad(p1)
            p2 = np.deg2rad(p2)
        msg = CmdSetMotorPosition()
        msg.cmds.append(self.create_SetMotorPosition(51, p1, 0.3, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(52, p2, 0.3, 0.5))
        self.leg_cmd_pos.publish(msg)
        time.sleep(delay)

    def set_height(self, h, delay=5.0):
        L = 240
        h = max(150, min(h, 450))
        a = np.deg2rad(60) - np.arccos(h / 2 / L)

        self.set_waist_pos(self.waist_status.motors[0].pos, a, delay=0.0)
        self.set_leg_pos(-a, a, delay=0.0)
        time.sleep(delay)
    
    def get_height(self):
        L = 240
        a = np.deg2rad(60) - self.leg_status.motors[1].pos
        h = np.cos(a) * L * 2
        return h

    def set_arm_home_pose(self, delay=5.0):
        pos_L = [0.64, -0.15, 0.0, -1.31, 0.0, -0.48, -0.35]
        pos_R = [0.64, 0.15, 0.0, -1.31, 0.0, -0.48, 0.35]
        self.arm.set_arm_joints(pos_L, pos_R, delay)

    def set_home_pose(self, head=True, body=True, arm=True, hand=True, delay=5.0):
        if head: self.set_head_pos((0.0, 0.0, 0.0), delay=0.0)
        if hand: self.set_left_hand(1.0, 1.0, 1.0, delay=0.0)
        if hand: self.set_right_hand(1.0, 1.0, 1.0, delay=0.0)
        if body: self.set_height(240, delay=0.0)
        if arm: self.set_arm_home_pose(delay=delay)
    
    def set_left_hand(self, angle1, angle2, angle3, force=0.1, delay=3.0):
        request = SetForce.Request()
        request.force0_ratio = force
        request.force1_ratio = force
        request.force2_ratio = force 
        request.force3_ratio = force 
        request.force4_ratio = force 
        request.force5_ratio = force 
        self.set_force_left_hand.call_async(request)

        request = SetAngleFlexible.Request()
        request.name = ["1", "2", "3", "4", "5", "6"]
        request.angle_ratio = [angle1, angle1, angle1, angle1, angle2, angle3]
        self.set_angle_left_hand.call_async(request)
        time.sleep(delay)
    
    def set_right_hand(self, angle1, angle2, angle3, force=0.1, delay=3.0):
        request = SetForce.Request()
        request.force0_ratio = force
        request.force1_ratio = force
        request.force2_ratio = force 
        request.force3_ratio = force 
        request.force4_ratio = force 
        request.force5_ratio = force 
        self.set_force_right_hand.call_async(request)

        request = SetAngleFlexible.Request()
        request.name = ["1", "2", "3", "4", "5", "6"]
        request.angle_ratio = [angle1, angle1, angle1, angle1, angle2, angle3]
        self.set_angle_right_hand.call_async(request)
        time.sleep(delay)

    def check_left_hand(self, delay=10.0):
        print("Check Left hand.")
        request = SetGestureForceCalibration.Request()
        self.set_gesture_force_calibration_L.call_async(request)
        time.sleep(delay)

    def check_right_hand(self, delay=10.0):
        print("Check right hand.")
        request = SetGestureForceCalibration.Request()
        self.set_gesture_force_calibration_R.call_async(request)
        time.sleep(delay)

    def say(self, text, delay=3.0):
        self.get_logger().info(text)
        request = PlayText.Request()
        request.text = text
        self.play_text.call_async(request)
        time.sleep(delay)

    def play(self, audio_path, delay=5.0):
        self.get_logger().info(audio_path)
        request = PlayFile.Request()
        request.path = audio_path 
        self.play_file.call_async(request)
        time.sleep(delay)
        request = PlayStop.Request()
        self.play_stop.call_async(request)

    def is_moving(self):
        e = 0.0
        for i in range(3):
            e += (self.head_status.motors[i].pos - self.head_status.motors_pre[i].pos) ** 2
        for i in range(2):
            e += (self.waist_status.motors[i].pos - self.waist_status.motors_pre[i].pos) ** 2
        for i in range(2):
            e += (self.leg_status.motors[i].pos - self.leg_status.motors_pre[i].pos) ** 2
        for i in range(14):
            e += (self.arm_status.motors[i].pos - self.arm_status.motors_pre[i].pos) ** 2
        print("error:", e)
        if e > 0.0001:
            self.ec = 0
        else:
            self.ec += 1
        return self.ec > 10
    
