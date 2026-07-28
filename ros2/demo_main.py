from KindaGarten.basic import *
from KindaGarten import *


if __name__ == "__main__":
    rclpy.init()

    node_yolo = YoloSubscriber(
        topic_image="/ob_camera_head/yolo/image",
        topic_depth="/ob_camera_head/yolo/depth",
        topic_yolo="/ob_camera_head/yolo/obj"
    )
    node_slamtec = SlamtecNode(ip="192.168.11.1", port=1448)
    node_ollama = OllamaNode(host="192.168.41.2:11434")
    node_robot = RobotNode()

    while rclpy.ok():
        t1 = time.time()
        SingleThreadedExecutorInstance().spin_once()

        if not node_yolo.ok(): print("waiting..."); time.sleep(1); continue
        image = node_yolo.image.copy()
        depth = node_yolo.depth.copy()

        for obj in node_yolo.yolo_data["obj"]:
            x1, y1, x2, y2 = obj["xyxy"]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if "xy" in obj:
                image = node_yolo.draw_mask_by_xy(image, obj["xy"])

        # print(node_slamtec.get_power_status())
        # print(node_ollama.send_message("hi"))
        # print(node_robot.head_status)
        
        cv2.imshow("frame", node_yolo.get_display_image(image, depth))
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break       

        time.sleep(max(0.0, min(0.05 - (time.time() - t1), 0.05)))
        print(time.time() - t1)
