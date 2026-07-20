import rclpy
from xarm_sdk import XARM_manager, TopicPublisher, ParamConfiger
from eai_manipulator_msgs.action import EndPosSingleTarget
from xarm_sdk.tools import action_caller
from xarm_sdk.utils import *
from geometry_msgs.msg import Pose
import time 
import numpy as np 
from scipy.spatial.transform import Rotation as R

# def euler_to_quaternion(rx, ry, rz):
#     rpy = np.deg2rad([rx, ry, rz])
#     rot = R.from_euler('xyz', rpy)
#     q = rot.as_quat()
#     return q


class XarmHandler(object):
    def __init__(self):
        self.xarm = XARM_manager()
        self.xarm.xarm_deactivate_all_controller()
        self.xarm.get_logger().info("deactivate all controller.")
        self.topic_publisher = TopicPublisher(self.xarm)

        param_configer = ParamConfiger(self.xarm)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_L_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_single_arm_qp_R_controller", "ori_err_bound", 3.0)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "dis_err_bound", 10.0)
        param_configer.set_node_parameter("endpose_dual_arm_qp_controller", "ori_err_bound", 3.0)

        self.xarm.xarm_activate_controller("endpose_single_arm_qp_L_controller")
        self.xarm.get_logger().info("activate endpose_single_arm_qp_L_controller")
        self.xarm.xarm_activate_controller("endpose_single_arm_qp_R_controller")
        self.xarm.get_logger().info("activate endpose_single_arm_qp_R_controller")
        self.xarm.xarm_activate_controller("endpose_dual_arm_qp_controller")
        self.xarm.get_logger().info("activate endpose_dual_arm_qp_controller")

    def set_endpose_L(self, x, y, z, rx, ry, rz):
        msg_pose = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose.position.x = x
        msg_pose.position.y = y
        msg_pose.position.z = z
        rpy = np.deg2rad([rx, ry, rz])

        q = euler_to_quaternion(*rpy)
        msg_pose.orientation.x = q[0]
        msg_pose.orientation.y = q[1]
        msg_pose.orientation.z = q[2]
        msg_pose.orientation.w = q[3]

        self.topic_publisher.publish_endposetarget_L(msg_pose, from_frame="waist_yaw_link")
        time.sleep(3)
    
    def set_endpose_R(self, x, y, z, rx, ry, rz):
        msg_pose = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose.position.x = x
        msg_pose.position.y = y
        msg_pose.position.z = z
        rpy = np.deg2rad([rx, ry, rz])

        q = euler_to_quaternion(*rpy)
        msg_pose.orientation.x = q[0]
        msg_pose.orientation.y = q[1]
        msg_pose.orientation.z = q[2]
        msg_pose.orientation.w = q[3]

        self.topic_publisher.publish_endposetarget_R(msg_pose, from_frame="waist_yaw_link")
        time.sleep(3)

    def set_endpose_LR(self, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2):
        msg_pose1 = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose1.position.x = x1
        msg_pose1.position.y = y1
        msg_pose1.position.z = z1
        rpy = np.deg2rad([rx1, ry1, rz1])

        q = euler_to_quaternion(*rpy)
        msg_pose1.orientation.x = q[0]
        msg_pose1.orientation.y = q[1]
        msg_pose1.orientation.z = q[2]
        msg_pose1.orientation.w = q[3]

        msg_pose2 = Pose()
        # # 设置左臂初始位置（单位：米）
        msg_pose2.position.x = x2
        msg_pose2.position.y = y2
        msg_pose2.position.z = z2
        rpy = np.deg2rad([rx2, ry2, rz2])

        q = euler_to_quaternion(*rpy)
        msg_pose2.orientation.x = q[0]
        msg_pose2.orientation.y = q[1]
        msg_pose2.orientation.z = q[2]
        msg_pose2.orientation.w = q[3]

        self.topic_publisher.publish_endposetarget_dual(msg_pose1, msg_pose2, from_frame_l="waist_yaw_link", from_frame_r="waist_yaw_link")
        time.sleep(3)


def main():
    rclpy.init()
    
    arm = XarmHandler()
    # arm.set_endpose_L(0.2, 0.4, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_L(0.3, 0.3, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_L(0.4, 0.2, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_L(0.5, 0.2, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_L(0.6, 0.2, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_L(0.6, 0.1, 0.0, 0.0, -90, 0.0)

    # arm.set_endpose_R(0.2, -0.4, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_R(0.3, -0.3, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_R(0.4, -0.2, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_R(0.5, -0.2, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_R(0.6, -0.2, 0.0, 0.0, -90, 0.0)
    # arm.set_endpose_R(0.6, -0.1, 0.0, 0.0, -90, 0.0)

    arm.set_endpose_LR(0.2, 0.4, 0.0, 0.0, -90, 0.0, 0.2, -0.4, 0.0, 0.0, -90, 0.0)
    arm.set_endpose_LR(0.3, 0.3, 0.0, 0.0, -90, 0.0, 0.3, -0.3, 0.0, 0.0, -90, 0.0)
    arm.set_endpose_LR(0.4, 0.2, 0.0, 0.0, -90, 0.0, 0.4, -0.2, 0.0, 0.0, -90, 0.0)
    arm.set_endpose_LR(0.5, 0.2, 0.0, 0.0, -90, 0.0, 0.5, -0.2, 0.0, 0.0, -90, 0.0)
    arm.set_endpose_LR(0.6, 0.2, 0.0, 0.0, -90, 0.0, 0.6, -0.2, 0.0, 0.0, -90, 0.0)
    arm.set_endpose_LR(0.6, 0.1, 0.0, 0.0, -90, 0.0, 0.6, -0.1, 0.0, 0.0, -90, 0.0)


if __name__ == "__main__":
    main()
