#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .egomotion import EgoMotion



class EgomotionSolver(Node):
    def __init__(self):
        super().__init__("egomotion_solver")

        self.bridge = CvBridge()
        self.frame  = None

        qos = rclpy.qos.QoSProfile(depth=1)
        self.sub_left  = self.create_subscription(Image,
                                                  "/cam0/image_raw",
                                                  self.on_image,
                                                  qos)

        self.publisherPOS_ = self.create_publisher(PoseStamped, '/CAMpose', 10)

        self.ego = EgoMotion(framewidth=640, frameheight=480, fps=15, calibFile="data/calibrationL.calib")

        # main draw loop – fires even if only one camera publishes for a while
        self.timer = self.create_timer(1.0 / 30, self.calculatepose)   # 15 Hz refresh
        self.get_logger().info("EgoMotion Started")

    # ---------- callbacks -----------------------------------------------------
    def on_image(self, msg: Image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Left decode error: {e}")

    # ---------- UI loop -------------------------------------------------------
    def calculatepose(self):
        if self.frame is None:
            return  # wait until frame arrives atleast once

        self.ego.update_frames(self.frame)
        ret = self.ego.optical_flow()

        if ret:
            self.ego.calculate_egomotion(drawpoints=False, showtR=False)

        td = self.ego.current_location().flatten()
        Rd = self.ego.current_rotation()

        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/cam_pose_raw"
        msg.pose.position.x = td[0]
        msg.pose.position.y = td[1]
        msg.pose.position.z = td[2]
        msg.pose.orientation.x = Rd[0]
        msg.pose.orientation.y = Rd[1]
        msg.pose.orientation.z = Rd[2]
        msg.pose.orientation.w = Rd[3]

        self.publisherPOS_.publish(msg)

    # ---------- clean-up ------------------------------------------------------
    def destroy_node(self):
        super().destroy_node()


# -----------------------------------------------------------------------------
def main():
    rclpy.init()
    egoapp = EgomotionSolver()
    try:
        rclpy.spin(egoapp)
    except KeyboardInterrupt:
        pass
    finally:
        egoapp.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
