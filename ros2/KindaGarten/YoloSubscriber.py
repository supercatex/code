from basic import *


class YoloSubscriber(Node):
    def __init__(self, topic_image: str, topic_depth: str, topic_yolo: str):
        super().__init__("YoloSubscriber")
        self.topic_image = topic_image
        self.topic_depth = topic_depth 
        self.topic_yolo = topic_yolo

        self.sub_image = SubscriptionNode(
            Image, topic_image, 
            lambda x: CvBridge().imgmsg_to_cv2(x, "bgr8")
        )
        self.sub_depth = SubscriptionNode(
            Image, topic_depth,
            lambda x: CvBridge().imgmsg_to_cv2(x)
        )
        self.sub_yolo = SubscriptionNode(
            String, topic_yolo,
            lambda x: json.loads(x.data)
        )

    def ok(self)->bool:
        if self.sub_image.data is None: return False 
        if self.sub_depth.data is None: return False
        if self.sub_yolo.data is None: return False 
        return True 

    @property
    def image(self):
        return self.sub_image.data

    @property
    def depth(self):
        return self.sub_depth.data 

    @property
    def yolo_data(self):
        return self.sub_yolo.data


if __name__ == "__main__":
    rclpy.init()
    node = YoloSubscriber(
        topic_image="/ob_camera_head/yolo/image",
        topic_depth="/ob_camera_head/yolo/depth",
        topic_yolo="/ob_camera_head/yolo/obj"
    )

    while rclpy.ok():
        t1 = time.time()
        SingleThreadedExecutorInstance().spin_once()

        if not node.ok(): print("waiting..."); time.sleep(1); continue
        image = node.sub_image.data.copy()
        depth = node.sub_depth.data.copy()

        for obj in node.yolo_data["obj"]:
            x1, y1, x2, y2 = obj["xyxy"]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if "xy" in obj:
                b_mask = np.zeros(image.shape[:2], np.uint8)
                contour = np.array(obj["xy"], dtype=np.int32).reshape(-1, 1, 2)
                cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)

                color3ch = np.zeros(image.shape, dtype=np.uint8)
                color3ch[:, :, 1] = 255
                mask3ch = cv2.cvtColor(b_mask, cv2.COLOR_GRAY2BGR)
                isolated = cv2.bitwise_and(mask3ch, color3ch)
                cv2.addWeighted(isolated, 0.2, image, 1.0, 0, image)

        h, w, c = image.shape
        h, w = h // 2, w // 2
        heatmap = np.array(depth / np.max(depth) * 255, dtype=np.uint8)
        heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)
        heatmap = cv2.resize(heatmap, (w, h))
        display = cv2.resize(image, (w, h))
        cv2.imshow("frame", cv2.hconcat([display, heatmap]))
        key_code = cv2.waitKey(1)
        if key_code in [27, ord('q')]:
            break

        time.sleep(max(0.0, min(0.05 - (time.time() - t1), 0.05)))
        node.get_logger().info(f"{time.time() - t1}")

    node.destroy_node()
    rclpy.shutdown()
