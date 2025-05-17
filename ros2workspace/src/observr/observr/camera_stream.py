#!/usr/bin/env python3
#
# Subscribes to /cam_left/image_raw  and  /cam_right/image_raw,
# converts each sensor_msgs/Image to OpenCV BGR, and shows a
# live side-by-side preview.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class StereoViewer(Node):
    def __init__(self):
        super().__init__("stereo_viewer")

        self.bridge = CvBridge()
        self.left_frame  = None
        self.right_frame = None

        qos = rclpy.qos.QoSProfile(depth=5)
        self.sub_left  = self.create_subscription(Image,
                                                  "/cam0/image_raw",
                                                  self.on_left,
                                                  qos)
        self.sub_right = self.create_subscription(Image,
                                                  "/cam1/image_raw",
                                                  self.on_right,
                                                  qos)

        # main draw loop – fires even if only one camera publishes for a while
        self.timer = self.create_timer(1.0 / 60.0, self.draw)   # 60 Hz UI refresh
        self.get_logger().info("Stereo viewer started")

    # ---------- callbacks -----------------------------------------------------
    def on_left(self, msg: Image):
        try:
            self.left_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Left decode error: {e}")

    def on_right(self, msg: Image):
        try:
            self.right_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Right decode error: {e}")

    # ---------- UI loop -------------------------------------------------------
    def draw(self):
        if self.left_frame is None or self.right_frame is None:
            return  # wait until both arrive at least once

        canvas = np.concatenate((self.left_frame, self.right_frame), axis=1)
        cv2.imshow("Stereo preview    'q' to quit", canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # exit cleanly
            rclpy.shutdown()

    # ---------- clean-up ------------------------------------------------------
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


# -----------------------------------------------------------------------------
def main():
    rclpy.init()
    viewer = StereoViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
