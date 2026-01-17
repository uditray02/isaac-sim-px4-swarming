#!/usr/bin/env python3

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

from ultralytics import YOLO


class Yolo26ROS2(Node):
    def __init__(self):
        super().__init__("yolo26_ros2_node")

        self.bridge = CvBridge()

        # ---------------- YOLO26 CONFIG ----------------
        self.model_path = "yolo26s.pt"   # or yolo26n.pt
        self.conf = 0.5
        self.classes = None   # set [0] for person only

        # ---------------- LOAD MODEL ----------------
        self.get_logger().info("Loading YOLO26 model...")
        self.model = YOLO(self.model_path)

        # ---------------- ROS SUB ----------------
        self.subscription = self.create_subscription(
            Image,
            "/drone1/camera/color/image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info("YOLO26 ROS2 node started")

    # --------------------------------------------------
    def image_callback(self, msg):
        # ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # ---------------- YOLO26 INFERENCE ----------------
        results = self.model.predict(
            source=frame,
            conf=self.conf,
            classes=self.classes,
            verbose=False
        )

        r = results[0]

        # ---------------- DRAW ----------------
        if r.boxes is not None and len(r.boxes) > 0:
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            clss  = r.boxes.cls.cpu().numpy()

            for (x1, y1, x2, y2), conf, cls in zip(boxes, confs, clss):
                c = int(cls)
                label = f"{self.model.names[c]} {conf:.2f}"

                cv2.rectangle(
                    frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2
                )
                cv2.putText(
                    frame,
                    label,
                    (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

        cv2.imshow("YOLO26 Detection", frame)
        cv2.waitKey(1)


# --------------------------------------------------
def main():
    rclpy.init()
    node = Yolo26ROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
