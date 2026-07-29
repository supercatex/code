from KindaGarten.basic import *
from KindaGarten import *


if __name__ == "__main__":
    rclpy.init()    # rospy.init_node("demo") ROS1

    node_yolo = YoloSubscriber(
        topic_image="/ob_camera_head/yolo/image",
        topic_depth="/ob_camera_head/yolo/depth",
        # topic_image="/ob_camera_head/color/image_raw",
        # topic_depth="/ob_camera_head/depth/image_raw",
        topic_yolo="/ob_camera_head/yolo/obj"
    )
    node_slamtec = SlamtecNode(ip="192.168.11.1", port=1448)
    node_ollama = OllamaNode(host="192.168.41.2:11434")
    node_robot = RobotNode()

    node_robot.play("/home/nvidia/data/speech/builtin/anjianyin.mp3", delay=1.0)
    node_robot.say("大家好")

    node_robot.say("重置機器人姿態")
    node_robot.arm.deactivate()
    node_robot.set_home_pose(delay=1.0)
    node_robot.set_height(150, delay=0.0)
    node_robot.set_head_pos((0.0, -20.0, 0.0), deg=True)

    node_robot.say("正在截入地圖並設置初始點")
    node_slamtec.set_stcm_map("/home/ubuntu/workspace/map_002.stcm")
    node_slamtec.set_localization_pose(0.0, 0.0, 0.0)

    state, next_state = "ready", ""
    state_t1 = time.time()
    action = None 

    fps = 15
    while rclpy.ok():
        t1 = time.time()
        SingleThreadedExecutorInstance().spin_once()
        if not node_yolo.ok(): print("waiting..."); time.sleep(1); continue
        image = node_yolo.image.copy()
        depth = node_yolo.depth.copy()
        yolo_data = node_yolo.yolo_data.copy()

        if state == "ready":
            node_robot.say("我準備好了！準備出發去目的地！")
            p1 = node_slamtec.get_poi_by_name("P1")
            action = node_slamtec.move_to(*p1)
            state, next_state = "move_to", "detect_bottle"
        elif state == "move_to":
            status = node_slamtec.get_action_status(action["action_id"])
            # print("\r", status, end="")
            if status["state"]["status"] == 4:
                node_robot.play("/home/nvidia/data/speech/builtin/anjianyin.mp3", delay=1)
                node_robot.say("到達目的地")
                state = next_state
                state_t1 = time.time()
        elif state == "go_home":
            status = node_slamtec.get_power_status()
            # print("\r", status, end="")
            if status["dockingStatus"] == "on_dock":
                node_robot.play("/home/nvidia/data/speech/builtin/anjianyin.mp3", delay=1)
                node_robot.say("正在充電中")
                state = next_state
                state_t1 = time.time()
        elif state == "refresh_camera":
            if time.time() - state_t1 > 5:
                state = next_state
                state_t1 = time.time()
        elif state == "detect_bottle":
            time.sleep(1)
            tx, ty, tz = -1, -1, -1
            for obj in yolo_data["obj"]:
                if obj["cls"] != 39: continue
                if obj["conf"] < 0.5: continue
                x1, y1, x2, y2 = obj["xyxy"]
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                x, y, z = get_real_xyz(cx, cy, depth, node_robot.head_status.rpy.tolist(), 0)
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(image, f"{x:.2f}, {y:.2f}, {z:.2f}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(x, y, z, obj["depth"])
                if x == 0: continue 
                if tx == -1 or x < tx:
                    tx, ty, tz = x, y, z 

            if tx != -1:
                x = min(max(0.3, tx - 0.2), 0.8)
                print(tx, ty, tz)
                print(x, ty, tz)
                if y > 0:
                    node_robot.arm.set_endpose_L((0.0, 0.4, tz), (0.0, -90, 0.0), deg=True, delay=5)
                    node_robot.arm.set_endpose_L((x, ty, tz), (0.0, -90, 0.0), deg=True, delay=5)
                    node_robot.arm.set_endpose_L((0.0, 0.4, tz), (0.0, -90, 0.0), deg=True, delay=5)
                else:
                    node_robot.arm.set_endpose_R((0.0, -0.4, z), (0.0, -90, 0.0), deg=True, delay=5)
                    node_robot.arm.set_endpose_R((x, ty, tz), (0.0, -90, 0.0), deg=True, delay=5)
                    node_robot.arm.set_endpose_R((0.0, -0.4, tz), (0.0, -90, 0.0), deg=True, delay=5)

            if time.time() - state_t1 > 60:
                node_robot.set_home_pose()
                node_robot.set_height(150, delay=0.0)
                node_robot.set_head_pos((0.0, -20.0, 0.0), deg=True)
                
                node_robot.say("回家了")
                node_slamtec.move_to_home()
                state, next_state = "go_home", "home"
            else:
                node_robot.say("再來一次")
                state, next_state = "refresh_camera", "detect_bottle"
                state_t1 = time.time()
        elif state == "home":
            node_robot.say("任務完成")
            break

        cv2.imshow("frame", node_yolo.get_display_image(image, depth))
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break       

        time.sleep(max(0.0, min(1/fps - (time.time() - t1), 1/fps)))
        # print(time.time() - t1)
