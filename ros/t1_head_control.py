import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import time 
from ultralytics import YOLO
from booster_robotics_sdk_python import ChannelFactory, B1LowCmdPublisher, LowCmd, LowCmdType, MotorCmd, B1JointCnt, B1JointIndex
import numpy as np 
from sensor_msgs.msg import JointState
from booster_robotics_sdk_python import B1LowStateSubscriber


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__(__class__.__name__)
        self.image = None
        self.depth = None
        self.sub_image = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.callback_image,
            10
        )
        self.sub_depth = self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.callback_depth,
            10
        )
        self.cb = CvBridge()

    def callback_image(self, msg):
        self.image = self.cb.imgmsg_to_cv2(msg, "bgr8")

    def callback_depth(self, msg):
        self.depth = self.cb.imgmsg_to_cv2(msg, "passthrough")


def handler(low_state_msg):
    global _head_pose
    _head_pose["yaw"] = low_state_msg.motor_state_parallel[0]
    _head_pose["pitch"] = low_state_msg.motor_state_parallel[1]

    # print("Received message:")
    # print(f"  serial motor count: {len(low_state_msg.motor_state_serial)}")
    # print(f"  parallel motor count: {len(low_state_msg.motor_state_parallel)}")
    # imu_state = low_state_msg.imu_state
    # print(f"  imu: {imu_state.rpy[0]}, {imu_state.rpy[1]}, {imu_state.rpy[2]}, "
    #       f"{imu_state.gyro[0]}, {imu_state.gyro[1]}, {imu_state.gyro[2]}, "
    #       f"{imu_state.acc[0]}, {imu_state.acc[1]}, {imu_state.acc[2]}")
    # for i, motor in enumerate(low_state_msg.motor_state_serial):
    #     print(f"  serial motor {i}: {motor.dq}, {motor.ddq}, {motor.tau_est}")
    # for i, motor in enumerate(low_state_msg.motor_state_parallel):
    #     print(
    #         f"  parallel motor {i}: {motor.dq}, {motor.ddq}, {motor.tau_est}")
    # print("done")


def move_head(pitch, yaw):
    global channel_publisher
    pitch = max(-18 * np.pi / 180, min(pitch, 48 * np.pi / 180))
    yaw = max(-58 * np.pi / 180, min(yaw, 58 * np.pi / 180))

    low_cmd = LowCmd()
    low_cmd.cmd_type = LowCmdType.SERIAL
    low_cmd.motor_cmd = [MotorCmd() for _ in range(B1JointCnt)]
    for i in range(B1JointCnt):
        low_cmd.motor_cmd[i].q = 0.0
        low_cmd.motor_cmd[i].dq = 0.0
        low_cmd.motor_cmd[i].tau = 0.0
        low_cmd.motor_cmd[i].kp = 0.0
        low_cmd.motor_cmd[i].kd = 0.0
        low_cmd.motor_cmd[i].weight = 0.0
        if i == B1JointIndex.kHeadPitch.value:
            low_cmd.motor_cmd[i].q = pitch
            low_cmd.motor_cmd[i].dq = 0.0
            low_cmd.motor_cmd[i].tau = 0.0
            low_cmd.motor_cmd[i].kp = 4.0
            low_cmd.motor_cmd[i].kd = 1.0
            low_cmd.motor_cmd[i].weight = 1.0
        if i == B1JointIndex.kHeadYaw.value:
            low_cmd.motor_cmd[i].q = yaw
            low_cmd.motor_cmd[i].dq = 0.0
            low_cmd.motor_cmd[i].tau = 0.0
            low_cmd.motor_cmd[i].kp = 4.0
            low_cmd.motor_cmd[i].kd = 1.0
            low_cmd.motor_cmd[i].weight = 1.0
    channel_publisher.Write(low_cmd)


if __name__ == "__main__":
    rclpy.init()
    sub_camera = CameraSubscriber()

    _head_pose = {}

    ChannelFactory.Instance().Init(0)
    channel_subscriber = B1LowStateSubscriber(handler)
    channel_subscriber.InitChannel()
    channel_publisher = B1LowCmdPublisher()
    channel_publisher.InitChannel()

    model_pose = YOLO("yolo11n-pose.pt")
    for i in range(20): move_head(0, 0)

    time.sleep(1)
    t1 = time.time()
    while rclpy.ok():
        time.sleep(max(0, 1.0 / 10 - t1))
        rclpy.spin_once(sub_camera)
        image, depth = sub_camera.image, sub_camera.depth
        if image is None or depth is None: continue

        h, w, c = image.shape
        res = model_pose(image, verbose=False)[0]
        cx, cy = w // 2, h // 2
        for p in res.keypoints.xy:
            if len(p) == 0: continue
            x0, y0 = map(int, (p[0][0], p[0][1]))
            cv2.circle(image, (x0, y0), 3, (0, 255, 0), -1)
            cx, cy = x0, y0
        
        p1 = 0.3 / (w // 2)
        e1 = w // 2 - cx
        v1 = p1 * e1
        p2 = 0.3 / (h // 2)
        e2 = h // 2 - cy
        v2 = p2 * -e2 
        move_head(_head_pose["pitch"].q + v2, _head_pose["yaw"].q + v1)

        # cv2.imwrite("demo.jpg", image)
        # print("FPS:", 1.0 / (time.time() - t1))
        t1 = time.time()

    sub_camera.destroy_node()
    rclpy.shutdown()
