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
from xarm_sdk import XARM_manager, TopicPublisher, ParamConfiger
from xarm_sdk.utils import *
from geometry_msgs.msg import Pose
import time 
from bodyctrl_msgs.srv import SetAngleFlexible, SetForce, SetGestureForceCalibration
from lyre_msgs.srv import PlayText, PlayFile, PlayStop
import requests


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
        return "".join([str(x) for x in self.motors])

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
        return "".join([str(x) for x in self.motors])

class HandStatus(object):
    def __init__(self, msg=None):
        self.msg = msg 
        self.motors: list[MotorStatus] = []
        for i in range(6): self.motors.append(MotorStatus())
        self.update(msg)
    
    def update(self, msg):
        if msg is None: return 
        for i in range(6):
            self.motors[i].update(msg.status[i])

    def __str__(self):
        return "".join([str(x) for x in self.motors])
    

class XarmHandler(object):
    def __init__(self):
        self.xarm = XARM_manager()
        self.deactivate()
        self.topic_publisher = TopicPublisher(self.xarm)

        param_configer = ParamConfiger(self.xarm)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "otg_p_step", 0.0005)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "otg_r_step", 0.0005)

        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "otg_p_step", 0.0005)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "otg_r_step", 0.0005)

        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "otg_p_step", 0.0005)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "otg_r_step", 0.0005)

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


class SlamtecNode(object):
    def __init__(self):
        self.ip = "192.168.11.1"
        self.port = 1448
        self.headers = {
            "accept": "application/json",
            "Content-Type": "application/json"
        }

    def send_request(self, api, method="GET", json_data={}):
        try:
            url = f"http://{self.ip}:{self.port}{api}"
            response = requests.request(
                method=method,
                url=url, 
                headers=self.headers,
                json=json_data
            )
            response.raise_for_status()
            return response.json()
        except Exception as e:
            print(e)
            return None
        
    def get_power_status(self):
        return self.send_request("/api/core/system/v1/power/status")
    
    def get_network_status(self):
        return self.send_request("/api/core/system/v1/network/status")

    def get_network_route(self):
        return self.send_request("/api/core/system/v1/network/route")
    
    def get_network_apn(self):
        return self.send_request("/api/core/system/v1/network/apn")
    
    def get_localization_pose(self):
        return self.send_request("/api/core/slam/v1/localization/pose")
    
    def get_localization_quality(self):
        return self.send_request("/api/core/slam/v1/localization/quality")
    
    def get_localization_odopose(self):
        return self.send_request("/api/core/slam/v1/localization/odopose")
        
    def get_localization_enable(self):
        return self.send_request("/api/core/slam/v1/localization/:enable")
    
    def get_mapping_enable(self):
        return self.send_request("/api/core/slam/v1/mapping/:enable")
    
    def get_loopclosure_enable(self):
        return self.send_request("/api/core/slam/v1/loopclosure/:enable")
    
    def get_point_of_interest(self):
        return self.send_request("/api/core/artifact/v1/pois")
    
    def get_action_factories(self):
        return self.send_request("/api/core/motion/v1/action-factories")
    
    def get_current_action(self):
        return self.send_request("/api/core/motion/v1/actions/:current")
    
    def move_to(self, x, y, yaw=0.0):
        return self.send_request("/api/core/motion/v1/actions", "POST", {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": {
                "target": {
                    "x": x,
                    "y": y,
                    "z": 0
                },
                "move_options": {
                    "mode": 2,
                    "flags": ["with_yaw"],
                    "yaw": yaw,
                    "acceptable_precision": 0,
                    "fail_retry_count": 0,
                    "speed_ratio": 0
                }
            }
        })
    
    def get_action_status(self, action_id):
        return self.send_request(f"/api/core/motion/v1/actions/{action_id}")
    
    def stop_current_action(self):
        return self.send_request("/api/core/motion/v1/actions/:current", "DELETE")
    
    def move_to_home(self):
        return self.send_request("/api/core/motion/v1/actions", "POST", {
            "action_name": "slamtec.agent.actions.GoHomeAction",
            "gohome_options": {
                "flags": "dock",
                "back_to_landing": True,
                "charging_retry_coount": 3
            }
        })
    
    def get_poi_by_name(self, name):
        res = self.get_point_of_interest()
        for obj in res:
            if obj["metadata"]["display_name"] == name:
                return obj["pose"]["x"], obj["pose"]["y"], obj["pose"]["yaw"]
        return None 

    def set_localization_pose(self, x, y, yaw):
        return self.send_request("/api/core/slam/v1/localization/pose", "PUT", {
            "x": x,
            "y": y,
            "z": 0,
            "yaw": yaw,
            "pitch": 0,
            "roll": 0
        })

    def set_localization_enable(self, enable):
        return self.send_request("/api/core/slam/v1/localization/:enable", "PUT", {
            "enable": enable
        })

    def set_mapping_enable(self, enable):
        return self.send_request("/api/core/slam/v1/mapping/:enable", "PUT", {
            "enable": enable
        })

    def set_loopclosure_enable(self, enable):
        return self.send_request("/api/core/slam/v1/loopclosure/:enable", "PUT", {
            "enable": enable
        })

    def reset_localization_status(self):
        return self.send_request("/api/core/slam/v1/localization/status/:reset", "POST")

    def set_stcm_map(self, filepath):
        with open(filepath, "rb") as f:
            try:
                url = f"http://{self.ip}:{self.port}/api/core/slam/v1/maps/stcm"
                response = requests.request(
                    method="PUT",
                    url=url, 
                    headers={
                        "accept": "*/*",
                        "Content-Type": "application/octet-stream"
                    },
                    data=f
                )
                response.raise_for_status()
                return True
            except Exception as e:
                print(e)
                return False
        return False

    def clear_stcm_map(self):
        return self.send_request("/api/core/slam/v1/maps", "DELETE")


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
        self.head_image = None #np.zeros((768, 1280, 3), dtype=np.uint8) 
        self.head_depth = None #np.zeros((768, 1280, 3), dtype=np.float32) 
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

        self.set_angle_left_hand = self.node.create_client(SetAngleFlexible, "/inspire_hand/set_angle_flexible/left_hand")
        self.set_angle_right_hand = self.node.create_client(SetAngleFlexible, "/inspire_hand/set_angle_flexible/right_hand")
        self.set_force_left_hand = self.node.create_client(SetForce, "/inspire_hand/set_force/left_hand")
        self.set_force_right_hand = self.node.create_client(SetForce, "/inspire_hand/set_force/right_hand")
        self.set_gesture_force_calibration_L = self.node.create_client(SetGestureForceCalibration, "/inspire_hand/set_gesture_force_calibration/left_hand")
        self.set_gesture_force_calibration_R = self.node.create_client(SetGestureForceCalibration, "/inspire_hand/set_gesture_force_calibration/right_hand")
        self.play_text = self.node.create_client(PlayText, "/audio_play/play_text")
        self.play_file = self.node.create_client(PlayFile, "/audio_play/play_file")
        self.play_stop = self.node.create_client(PlayStop, "/audio_play/stop")
        self.arm = XarmHandler()

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
        time.sleep(3)
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

    def set_head_pos(self, rpy, deg=False, delay=3.0):
        if deg: rpy = np.deg2rad(rpy)
        x, y, z = rpy

        msg = CmdSetMotorPosition()
        msg.cmds.append(self.create_SetMotorPosition(1, x, 1.0, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(2, y, 1.0, 0.5))
        msg.cmds.append(self.create_SetMotorPosition(3, z, 1.0, 0.5))
        self.head_cmd_pos.publish(msg)
        time.sleep(delay)

    def set_head_vel(self, x, y, z):
        msg = CmdSetMotorSpeed()
        msg.cmds.append(self.create_SetMotorSpeed(1, x, 0.5))
        msg.cmds.append(self.create_SetMotorSpeed(2, y, 0.5))
        msg.cmds.append(self.create_SetMotorSpeed(3, z, 0.5))
        self.head_cmd_vel.publish(msg)

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

    def set_home_pose(self, delay=5.0):
        self.set_head_pos((0.0, 0.0, 0.0), delay=0.0)
        self.set_left_hand(1.0, 1.0, 1.0, delay=0.0)
        self.set_right_hand(1.0, 1.0, 1.0, delay=0.0)
        self.set_height(240, delay=0.0)
        self.set_arm_home_pose(delay=delay)
    
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

    def say(self, text, delay=1.0):
        request = PlayText.Request()
        request.text = text
        self.play_text.call_async(request)
        time.sleep(delay)

    def play(self, filepath, delay=5.0):
        request = PlayFile.Request()
        request.path = filepath 
        self.play_file.call_async(request)
        time.sleep(delay)
        request = PlayStop.Request()
        self.play_stop.call_async(request)


class Main(Node):
    def __init__(self):
        super().__init__("demo")
        self.robot = X_Humanoid(self)

        self.timer = self.create_timer(1 / 20, self.callback_timer)

        self.robot.arm.deactivate()
        self.robot.set_home_pose()
        # input()
        self.base = SlamtecNode()

        self.robot.say("大家好")
        # self.robot.say("當前電量還有百分之%d" % self.base.get_power_status()["batteryPercentage"])
        self.robot.say("正在截入地圖並設置初始點")
        print(self.base.set_stcm_map("/home/ubuntu/workspace/map_001.stcm"))
        print(self.base.set_localization_pose(-2.74, 0.39, -0.05))
        time.sleep(3)
        
        self.robot.say("我準備好了，開始檢測鏡頭")
        time.sleep(3)

    def callback_timer(self):
        # if not self.robot.check_head(): return 

        self.robot.say("正在檢測頭部運動")
        self.robot.set_head_pos((0, 25, 0), deg=True)
        self.robot.set_head_pos((0, 0, 80), deg=True)
        self.robot.set_head_pos((0, -25, 0), deg=True)
        self.robot.set_head_pos((0, 0, -80), deg=True)
        self.robot.set_head_pos((0, 0, 0), deg=True)

        self.robot.say("正在檢測高度調節")
        self.robot.set_height(150)
        self.robot.set_height(450)
        self.robot.set_height(240)

        self.robot.say("彎腰測試")
        self.robot.set_waist_pos(0.0, 30, deg=True)
        self.robot.set_waist_pos(0.0, -30, deg=True)
        self.robot.set_waist_pos(0.0, 0.0)

        self.robot.say("展開雙臂")
        self.robot.arm.set_endpose_LR((0.2, 0.4, 0.0), (0.0, -90, 0.0), (0.2, -0.4, 0.0), (0.0, -90, 0.0), deg=True, delay=5.0)

        self.robot.say("舉高雙臂")
        self.robot.arm.set_endpose_LR((0.2, 0.4, 0.5), (0.0, -90, 0.0), (0.2, -0.4, 0.5), (0.0, -90, 0.0), deg=True, delay=5.0)

        self.robot.say("左臂測試")
        self.robot.arm.set_endpose_L((0.2, 0.4, 0.0), (0.0, -90, 0.0), deg=True, delay=5.0)

        self.robot.say("右臂測試")
        self.robot.arm.set_endpose_R((0.2, -0.4, 0.0), (0.0, -90, 0.0), deg=True, delay=5.0)

        self.robot.say("移動姿態")
        self.robot.set_home_pose()

        self.robot.say("檢測靈巧手")
        self.robot.check_left_hand(delay=0.0)
        self.robot.check_right_hand(delay=15.0)

        self.robot.say("左手抓取測試")
        self.robot.set_left_hand(1.0, 1.0, 1.0)
        self.robot.set_left_hand(1.0, 1.0, 0.0)
        self.robot.set_left_hand(0.6, 0.3, 0.0)

        self.robot.say("右手抓取測試")
        self.robot.set_right_hand(1.0, 1.0, 1.0)
        self.robot.set_right_hand(1.0, 1.0, 0.0)
        self.robot.set_right_hand(0.6, 0.3, 0.0)

        state = 0
        action = None
        while True:
            time.sleep(0.5)
            if state == 0:
                self.robot.say("正在前往P1")
                p1 = self.base.get_poi_by_name("P1")
                action = self.base.move_to(*p1)
                state = 1
            elif state == 1:
                action_state = self.base.get_action_status(action["action_id"])
                print(action_state)
                if action_state["state"]["status"] == 4:
                    self.robot.play("/home/nvidia/data/speech/sbus/100000.mp3", delay=10)
                    state = 2
            elif state == 2:
                self.robot.say("正在前往P2")
                p2 = self.base.get_poi_by_name("P2")
                action = self.base.move_to(*p2)
                state = 3
            elif state == 3:
                action_state = self.base.get_action_status(action["action_id"])
                print(action_state)
                if action_state["state"]["status"] == 4:
                    state = 4
            elif state == 4:
                self.robot.say("正在前往P3")
                p3 = self.base.get_poi_by_name("P3")
                action = self.base.move_to(*p3)
                state = 5
            elif state == 5:
                action_state = self.base.get_action_status(action["action_id"])
                print(action_state)
                if action_state["state"]["status"] == 4:
                    state = 6
            elif state == 6:
                self.robot.say("回家了")
                self.base.move_to_home()
                state = 7
            elif state == 7:
                action_state = self.base.get_action_status(action["action_id"])
                print(action_state)
                if action_state["state"]["status"] == 4:
                    break

        cv2.imshow("frame", self.robot.head_image)
        cv2.imshow("depth", self.robot.head_depth)
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            raise SystemExit
    

if __name__ == "__main__":
    rclpy.init()
    node = Main()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

