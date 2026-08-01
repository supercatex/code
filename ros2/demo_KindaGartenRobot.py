from KindaGarten.basic import *
from KindaGarten import *
from KindaGartenRobot import KindaGartenRobot


if __name__ == "__main__":
    rclpy.init()

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

    rclpy.shutdown()
