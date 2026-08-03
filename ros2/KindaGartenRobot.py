from KindaGarten.basic import *
from KindaGarten import *


class KindaGartenRobot(object):
    SUPPORT_STATE_NONE = 0
    SUPPORT_STATE_MOVE_TO_HOME = 1
    SUPPORT_STATE_MOVE_TO_POINT = 2
    SUPPORT_STATE_REFRESH_CAMERA = 3

    def __init__(self,):
        self.node_slamtec = SlamtecNode(ip="192.168.11.1", port=1448)
        self.node_ollama = OllamaNode(host="192.168.41.2:11434")
        self.node_robot = RobotNode()
        self.node_yolo = YoloSubscriber(
            topic_image="/ob_camera_head/yolo/image",
            topic_depth="/ob_camera_head/yolo/depth",
            topic_yolo="/ob_camera_head/yolo/obj"
        )

        self.fps = 15
        self.update_time = time.time()
        self.state = "init"
        self.next_state = ""
        self.prev_state = ""
        self.support_state = self.SUPPORT_STATE_NONE
        self.support_data = {}

        self.play_default()
        while not self.node_yolo.ok():
            self.update()
            time.sleep(1)
            self.play_default(7)
        self.play_default()

    def update(self):
        time.sleep(max(0.0, min(1/self.fps - (time.time() - self.update_time), 1/self.fps)))
        self.update_time = time.time()
        SingleThreadedExecutorInstance().spin_once()

        if self.SUPPORT_STATE_NONE:
            pass
        elif self.support_state == self.SUPPORT_STATE_MOVE_TO_HOME:
            status = self.node_slamtec.get_power_status()
            if status["dockingStatus"] == "on_dock":
                self.change_state(self.next_state)
                self.support_state = self.SUPPORT_STATE_NONE
        elif self.support_state == self.SUPPORT_STATE_MOVE_TO_POINT:
            action = self.support_data["action"]
            status = self.node_slamtec.get_action_status(action["action_id"])
            if status["state"]["status"] == 4:
                self.change_state(self.next_state)
                self.support_state = self.SUPPORT_STATE_NONE
        elif self.support_state == self.SUPPORT_STATE_REFRESH_CAMERA:
            if time.time() - self.support_data["start"] > self.support_data["delay"]:
                self.change_state(self.next_state)
                self.support_state = self.SUPPORT_STATE_NONE

    def change_state(self, state):
        self.prev_state = self.state 
        self.state = state 

    def display_image(self, image, depth):
        return self.node_yolo.get_display_image(image, depth)
    
# region property
    @property
    def image(self):
        return self.node_yolo.image

    @property
    def depth(self):
        return self.node_yolo.depth

    @property
    def yolo_data(self):
        return self.node_yolo.yolo_data["obj"]

    @property
    def head_rpy(self):
        return self.node_robot.head_status.rpy.tolist()

    @property
    def height(self):
        return self.node_robot.get_height()
# endregion property

# region​ AUDIO
    def play(self, audio_path, delay=3.0):
        self.node_robot.play(audio_path, delay)

    BUILTIN_AUDIO = {
        0: [1.31, "/home/nvidia/data/speech/builtin/anjianyin.mp3"],        # 開始自檢
        1: [3.27, "/home/nvidia/data/speech/builtin/chenggong.mp3"],        # 輕長短音
        2: [1.58, "/home/nvidia/data/speech/builtin/chenggong.wav"],        # 自檢完成
        3: [1.85, "/home/nvidia/data/speech/builtin/didianliang.wav"],      # 連咇兩聲
        4: [1.34, "/home/nvidia/data/speech/builtin/fuwudengdai.mp3"],      # 自檢音效
        5: [2.07, "/home/nvidia/data/speech/builtin/guzhang.mp3"],          # 緊急按鈕
        6: [10.2, "/home/nvidia/data/speech/builtin/kaijiqidong.mp3"],      # 開機音效
        7: [0.64, "/home/nvidia/data/speech/builtin/kaishichongdian.mp3"],  # 輕咇一聲
        8: [4.21, "/home/nvidia/data/speech/builtin/zijianguocheng.mp3"]    # 連咇幾聲
    }
    def play_default(self, audio_id=0, delay=None):
        duration, audio_path = self.BUILTIN_AUDIO[audio_id]
        if delay is not None: duration = delay
        self.play(audio_path, duration)
        time.sleep(0.1)

    def say(self, text, delay=3.0):
        self.node_robot.say(text, delay)
# endregion AUDIO

# region ROBOT
    def set_home_pose(self, head=True, body=True, arm=True, hand=True, delay=3.0):
        self.node_robot.set_home_pose(head, body, arm, hand, delay)

    def refresh_camera(self, delay, next_state):
        self.state = ""
        self.next_state = next_state
        self.support_state = self.SUPPORT_STATE_REFRESH_CAMERA
        self.support_data["delay"] = delay
        self.support_data["start"] = time.time()

    def set_head_pos(self, pos, deg=False, delay=3.0):
        self.node_robot.set_head_pos(pos, deg=deg, delay=delay)

    def set_height(self, value, delay=3.0):
        self.node_robot.set_height(value)

    def set_hand_L(self, pos_1, pos_2, pos_3, force=0.1, delay=1.0):
        self.node_robot.set_left_hand(pos_1, pos_2, pos_3, force, delay)

    def set_hand_R(self, pos_1, pos_2, pos_3, force=0.1, delay=1.0):
        self.node_robot.set_right_hand(pos_1, pos_2, pos_3, force, delay)

    def set_endpose_L(self, pos, rpy, deg=False, delay=5.0):
        self.node_robot.arm.set_endpose_L(pos, rpy, deg=deg, delay=delay)

    def set_endpose_R(self, pos, rpy, deg=False, delay=5.0):
        self.node_robot.arm.set_endpose_R(pos, rpy, deg=deg, delay=delay)

    def pick_up_L(self, pos, force=0.2):
        x, y, z = pos 
        rx = 20.0
        x = x - 0.1 * np.cos(np.deg2rad(rx))
        x = min(max(0.3, x), 1.0)
        xx = x - 0.1
        if y < 0:
            self.node_robot.get_logger().warn(f"{x}, {y}, {z} cannot use left hand.")
            return False
        
        self.set_endpose_L((0.1, 0.4, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_hand_L(1.0, 1.0, 0.0, delay=0)
        self.set_endpose_L((xx, y, z), (rx, -90, 0.0), deg=True, delay=5)
        self.set_endpose_L((x, y, z), (rx, -90, 0.0), deg=True, delay=2)
        self.set_hand_L(0.6, 0.5, 0.0, force=force, delay=1)
        self.set_endpose_L((0.1, 0.4, z + 0.1), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_endpose_L((0.1, 0.4, 0.0), (0.0, -90, 0.0), deg=True, delay=3)
        return True

    def pick_up_R(self, pos, force=0.2):
        x, y, z = pos 
        x = x - 0.075
        xx = min(max(0.3, x - 0.15), 1.0)
        rx = 20.0
        z = z - 0.05
        if y > 0:
            self.node_robot.get_logger().warn(f"{x}, {y}, {z} cannot use right hand.")
            return False
        
        self.set_endpose_R((0.0, -0.4, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_hand_R(1.0, 1.0, 0.0, delay=0)
        self.set_endpose_R((xx, y, z), (-rx, -90, 0.0), deg=True, delay=5)
        self.set_endpose_R((x, y, z), (-rx, -90, 0.0), deg=True, delay=2)
        self.set_hand_R(0.6, 0.5, 0.0, force=0.2, delay=1)
        self.set_endpose_R((0.1, -0.4, z + 0.1), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_endpose_R((0.1, -0.4, 0.0), (0.0, -90, 0.0), deg=True, delay=3)
        return True

    def put_down_L(self, pos):
        x, y, z = pos 
        if y < 0:
            self.node_robot.get_logger().warn(f"{x}, {y}, {z} cannot use left hand.")
            return False
        
        self.set_endpose_L((0.1, 0.4, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_endpose_L((x, y, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_hand_L(1.0, 1.0, 0.0, delay=2)
        self.set_endpose_L((0.1, 0.4, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_endpose_L((0.1, 0.4, 0.0), (0.0, -90, 0.0), deg=True, delay=3)
        return True

    def put_down_R(self, pos):
        x, y, z = pos 
        if y > 0:
            self.node_robot.get_logger().warn(f"{x}, {y}, {z} cannot use right hand.")
            return False

        self.set_endpose_R((0.1, -0.4, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_endpose_R((x, y, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_hand_R(1.0, 1.0, 0.0, delay=2)
        self.set_endpose_R((0.1, -0.4, z), (0.0, -90, 0.0), deg=True, delay=5)
        self.set_endpose_R((0.1, -0.4, 0.0), (0.0, -90, 0.0), deg=True, delay=3)
        return True
# endregion ROBOT

# region SLAMTEC
    def set_map_file(self, map_file):
        self.node_slamtec.set_stcm_map(map_file)
        
    def set_location(self, x, y, yaw):
        self.node_slamtec.set_localization_pose(x, y, yaw)

    def get_POI(self, name):
        return self.node_slamtec.get_poi_by_name(name)
                    
    def move_to_point(self, x, y, yaw, next_state, mode=2):
        # yaw => None: 任意朝向
        # mode => 0: 自由導航, 1: 嚴格軌道, 2: 軌道優先
        self.state = ""
        self.next_state = next_state
        self.support_state = self.SUPPORT_STATE_MOVE_TO_POINT
        self.support_data["action"] = self.node_slamtec.move_to(x, y, yaw, mode=mode)
        return self.support_data["action"]

    def move_to_POI(self, name, next_state, mode=2):
        x, y, yaw = self.get_POI(name)
        return self.move_to_point(x, y, yaw, next_state, mode)

    def move_to_home(self, next_state):
        self.state = ""
        self.next_state = next_state
        self.support_state = self.SUPPORT_STATE_MOVE_TO_HOME
        return self.node_slamtec.move_to_home()
# endregion SLAMTEC

def main():
    robot = KindaGartenRobot()

    use_LR = ""

    while rclpy.ok():
        robot.update()
        image = robot.image.copy()
        depth = robot.depth.copy()
        yolo_data = robot.yolo_data.copy()

    # init
        if robot.state == "init":
            robot.say("正在載入地圖", delay=0.0)
            robot.set_map_file("/home/ubuntu/workspace/map_002.stcm")
            robot.set_location(0, 0, 0)

            robot.say("調整到初始姿態", delay=0.0)
            robot.set_home_pose(delay=3.0)
            robot.change_state("state_1")
    # 前往P1
        elif robot.state == "state_1":
            robot.say("前往P1")
            robot.move_to_POI("P1", "state_2")
    # 刷新影像
        elif robot.state == "state_2":
            robot.say("到達P1")
            robot.say("調整高度和鏡頭角度")
            robot.set_head_pos((0, -20, 0), deg=True, delay=0)
            robot.set_height(150, delay=0)
            robot.say("刷新影像")
            robot.refresh_camera(5, "state_3")
    # 抓取水瓶
        elif robot.state == "state_3":
            tx, ty, tz = -1, -1, -1
            for obj in yolo_data:
                if obj["cls"] != 39: continue
                if obj["conf"] < 0.5: continue
                x1, y1, x2, y2 = obj["xyxy"]
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                x, y, z = get_real_xyz(cx, cy, depth)
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(image, f"{x:.2f}, {y:.2f}, {z:.2f}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if x != 0 and (tx == -1 or x < tx):
                    tx, ty, tz = x, y, z 
            print(tx, ty, tz)

            if tx != -1:
                if ty > 0:
                    robot.say("檢測到水瓶，現在用左手拿水瓶")
                    print("LEFT HAND", ty)
                    use_LR = "L"
                    robot.pick_up_L((tx, ty, tz), 0.2)
                else:
                    robot.say("檢測到水瓶，現在用右手拿水瓶")
                    print("RIGHT HAND", ty)
                    use_LR = "R"
                    robot.pick_up_R((tx, ty, tz), 0.2)
                robot.set_home_pose(hand=False)
                robot.change_state("state_4")
            else:
                robot.say("找不到水瓶了")
                robot.set_home_pose()
                robot.move_to_home("end")
    # 前往P2
        elif robot.state == "state_4":
            robot.say("前往P2")
            robot.move_to_POI("P2", "state_5")
    # 放置水瓶
        elif robot.state == "state_5":
            robot.say("到達P2")
            robot.say("現在放置水瓶")
            if use_LR == "L":
                robot.put_down_L((0.5, 0.2, 0.25))
            else:
                robot.put_down_R((0.5, -0.2, 0.25))
            robot.set_home_pose(delay=3.0)
            robot.change_state("state_1")
        elif robot.state == "end":
            robot.say("正在充電中 任務結束")
            break

        cv2.imshow("frame", robot.display_image(image, depth))
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break

if __name__ == "__main__":
    rclpy.init()
    main()
    rclpy.shutdown()
