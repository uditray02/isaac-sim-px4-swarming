#!/usr/bin/env python3

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch

# ---------------- PATH SETUP ----------------
FILE = Path(__file__).resolve()
ROOT = FILE.parent
sys.path.append(str(ROOT))

# YOLOv5 imports
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.plots import plot_one_box, colors
from utils.augmentations import letterbox
from utils.torch_utils import select_device

# -------------------------------------------


class YoloROS2(Node):

    def __init__(self):
        super().__init__('yolo_ros2_node')

        self.bridge = CvBridge()

        # ---------------- YOLO CONFIG ----------------
        self.weights = "yolov5s.pt"
        self.imgsz = 640
        self.conf_thres = 0.4
        self.iou_thres = 0.45
        self.max_det = 1000
        self.classes = None          # set [0] for person only
        self.agnostic_nms = False

        # ---------------- DEVICE ----------------
        self.device = select_device("0")
        self.half = self.device.type != 'cpu'

        # ---------------- LOAD MODEL ----------------
        self.model = attempt_load(self.weights, map_location=self.device)
        self.stride = int(self.model.stride.max())
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

        if self.half:
            self.model.half()

        # --------- CORRECT WARMUP (FIXED) ----------
        dummy = torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device)
        dummy = dummy.half() if self.half else dummy.float()
        self.model(dummy)
        # --------------------------------------------

        # ---------------- ROS SUB ----------------
        self.subscription = self.create_subscription(
            Image,
            "/drone1/camera/color/image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info("YOLOv5 ROS2 node started successfully")

    # --------------------------------------------------
    def image_callback(self, msg):

        img0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        img, ratio, pad = letterbox(
            img0,
            new_shape=self.imgsz,
            stride=self.stride,
            auto=True
        )

        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()
        img /= 255.0
        img = img.unsqueeze(0)

        with torch.no_grad():
            pred = self.model(img)[0]

        pred = non_max_suppression(
            pred,
            self.conf_thres,
            self.iou_thres,
            self.classes,
            self.agnostic_nms,
            max_det=self.max_det
        )

        for det in pred:
            if len(det):
                det[:, :4] = scale_coords(
                    img.shape[2:],
                    det[:, :4],
                    img0.shape
                ).round()

                for *xyxy, conf, cls in det:
                    c = int(cls)
                    label = f"{self.names[c]} {conf:.2f}"
                    plot_one_box(
                        xyxy,
                        img0,
                        label=label,
                        color=colors(c, True),
                        line_thickness=2
                    )

        # ---- DISPLAY FIX (THIS IS THE KEY) ----
        display_img = cv2.resize(img0, (960, 512), interpolation=cv2.INTER_AREA)
        cv2.imshow("Detection", display_img)
        cv2.waitKey(1)



# --------------------------------------------------
def main():
    rclpy.init()
    node = YoloROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


