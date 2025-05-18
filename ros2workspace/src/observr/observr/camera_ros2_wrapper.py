#!/usr/bin/env python3
#
# Publishes two Picamera2 streams on separate ROS 2 topics.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from picamera2 import Picamera2
from libcamera import controls
import cv2
import numpy as np


class DualCamPublisher(Node):
    def __init__(self,
                 size=(640, 480),            # (height, width)
                 framerate=30,               # ROS publish rate, not sensor FPS
                 sensor_fps=120):

        super().__init__("dual_camera_publisher")

        self.bridge = CvBridge()

        # --- open two cameras ------------------------------------------------
        self.cams = (Picamera2(1), Picamera2(0))

        cam_cfg = dict(
            controls={'FrameRate': sensor_fps,
                      'AfMode': controls.AfModeEnum.Manual,
                      'LensPosition': 1.1},
            main={'format': 'XRGB8888', 'size': size},   # WxH
            raw={'format': 'SRGGB10_CSI2P', 'size': (1536, 864)}
        )

        for cam in self.cams:
            cfg = cam.create_video_configuration(**cam_cfg)
            cam.configure(cfg)
            cam.start("main",show_preview=False)

        # --- ROS publishers --------------------------------------------------
        qos = rclpy.qos.QoSProfile(depth=5)
        self.pub_left =  self.create_publisher(Image, "/cam0/image_raw", qos)
        self.pub_right = self.create_publisher(Image, "/cam1/image_raw", qos)

        # --- timer -----------------------------------------------------------
        self.timer = self.create_timer(1.0 / framerate, self.grab_and_publish)
        self.get_logger().info("Dual camera node started")

    # -------------------------------------------------------------------------
    def grab_and_publish(self):
        try:
            
            # acquire   (make arrays contiguous to satisfy OpenCV/GTK)
            img0 = self.cams[0].capture_array()
            img1 = self.cams[1].capture_array()

            img0 = cv2.rotate(img0, cv2.ROTATE_90_COUNTERCLOCKWISE)
            img1 = cv2.rotate(img1, cv2.ROTATE_90_CLOCKWISE)

            # Picamera2 gives XRGB → convert to BGR (drop alpha)
            img_left  = cv2.cvtColor(img0,  cv2.COLOR_RGBA2RGB)
            img_right = cv2.cvtColor(img1, cv2.COLOR_RGBA2RGB)

            msg0 = self.bridge.cv2_to_imgmsg(img_left, encoding="rgb8")
            msg1 = self.bridge.cv2_to_imgmsg(img_right, encoding="rgb8")
            
            msg0.header = Header()
            msg0.header.stamp = self.get_clock().now().to_msg()
            msg0.header.frame_id = "/cam1_raw"

            msg1.header = Header()
            msg1.header.stamp = self.get_clock().now().to_msg()
            msg1.header.frame_id = "/cam1_raw"

            # publish
            self.pub_left.publish(msg0)
            self.pub_right.publish(msg1)

        except Exception as e:
            self.get_logger().error(f"Capture/publish error: {e}")

    # -------------------------------------------------------------------------
    def destroy_node(self):
        for cam in self.cams:
            cam.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = DualCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()