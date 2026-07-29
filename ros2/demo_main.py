from KindaGarten.basic import *
from KindaGarten import *


if __name__ == "__main__":
    rclpy.init()    # rospy.init_node("demo") ROS1

    node_yolo = YoloSubscriber(
        # topic_image="/ob_camera_head/yolo/image",
        # topic_depth="/ob_camera_head/yolo/depth",
        topic_image="/ob_camera_head/color/image_raw",
        topic_depth="/ob_camera_head/depth/image_raw",
        topic_yolo="/ob_camera_head/yolo/obj"
    )
    node_slamtec = SlamtecNode(ip="192.168.11.1", port=1448)
    node_ollama = OllamaNode(host="192.168.41.2:11434")
    node_robot = RobotNode()

    node_robot.play("/home/nvidia/data/speech/builtin/anjianyin.mp3", delay=1.0)
    node_robot.say("大家好")
    
    node_robot.arm.deactivate()
    node_robot.set_home_pose()

    node_robot.set_height(150)
    node_robot.set_head_pos((0.0, -20.0, 0.0), deg=True)

    fps = 15
    while rclpy.ok():
        t1 = time.time()
        SingleThreadedExecutorInstance().spin_once()

        if not node_yolo.ok(): print("waiting..."); time.sleep(1); continue
        image = node_yolo.image.copy()
        depth = node_yolo.depth.copy()
        yolo_data = node_yolo.yolo_data.copy()
        for obj in yolo_data["obj"]:
            if obj["cls"] != 39: continue
            if obj["conf"] < 0.5: continue
            x1, y1, x2, y2 = obj["xyxy"]
            # print(obj["cls"], obj["xyxy"], obj["conf"])
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
            x, y, z = get_real_xyz(cx, cy, depth, node_robot.head_status.rpy.tolist(), 0)
            cv2.putText(image, f"{x:.2f}, {y:.2f}, {z:.2f}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            # print(node_robot.head_status.rpy)
            print(x, y, z, obj["depth"])

            if x <= 0: continue
            if y > 0:
                node_robot.arm.set_endpose_L((0.4, y, z), (0.0, -90, 0.0), deg=True, delay=0.5)
            else:
                node_robot.arm.set_endpose_R((0.4, y, z), (0.0, -90, 0.0), deg=True, delay=0.5)
            # break

        # image = node_yolo.draw_boxes(image, node_yolo.yolo_data)

        # print(node_slamtec.get_power_status())
        # print(node_ollama.send_message("hi"))
        # print(node_robot.head_status)

        # cv2.imshow("frame", image)
        cv2.imshow("frame", node_yolo.get_display_image(image, depth))
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break       

        time.sleep(max(0.0, min(1/fps - (time.time() - t1), 1/fps)))
        print(time.time() - t1)
