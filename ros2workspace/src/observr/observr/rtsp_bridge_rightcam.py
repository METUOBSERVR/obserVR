#!/usr/bin/env python3
"""
Publish /cam1/image_raw as true RTSP:
    rtsp://<Pi_IP>:8554/cam
Works on Raspberry Pi 5 (Ubuntu 22.04 / 24.04 + ROS 2 Humble/Jazzy)
"""

import gi, rclpy, cv2, threading
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ---------- Camera parameters ----------
WIDTH  = 480
HEIGHT = 640
FPS    = 30

# ---------- ROS->RTSP bridge node ----------
class Cam1RTSP(Node):
    def __init__(self):
        super().__init__('rtsp_rightcam_server')
        self.bridge = CvBridge()
        self.latest = None            # last RGB frame bytes
        self.lock   = threading.Lock()

        # ROS subscription
        self.create_subscription(Image, '/cam1/image_raw', self.cb, 5)

        # GStreamer/RTSP init
        Gst.init(None)
        self.loop = GLib.MainLoop()
        threading.Thread(target=self.loop.run, daemon=True).start()
        self.setup_rtsp_server()
        self.get_logger().info("RTSP ready at rtsp://<Pi_IP>:8554/cam")

    # ---------- ROS callback ----------
    def cb(self, msg: Image):
        try:
            rgb  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            with self.lock:
                self.latest = rgb.tobytes()
        except Exception as e:
            self.get_logger().error(f"Bridge error: {e}")

    # ---------- RTSP server Pipeline ----------
    def setup_rtsp_server(self):
        server = GstRtspServer.RTSPServer()
        server.set_service("8554")

        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_shared(True)

        pipeline = (
            # incoming caps
            f'appsrc name=src is-live=true block=true do-timestamp=true '
            f'caps=video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS}/1 '
            # convert RGB → I420 (4:2:0) so baseline profile works
            '! videoconvert ! video/x-raw,format=I420 '
            # encode & packetise
            '! x264enc tune=zerolatency speed-preset=superfast key-int-max=30 '
            '! rtph264pay config-interval=1 name=pay0 pt=96'
        )
        factory.set_launch(pipeline)
        factory.connect('media-configure', self.on_configure)

        server.get_mount_points().add_factory("/cam", factory)
        server.attach(None)

    # ---------- Called when a viewer connects ----------
    def on_configure(self, factory, media):
        self.get_logger().info("Viewer connected")

        appsrc = media.get_element().get_child_by_name("src")
        appsrc.set_property("format", Gst.Format.TIME)
        appsrc.set_property("block", True)
        appsrc.set_property("do-timestamp", True)

        # push frames every 1/FPS seconds
        frame_period_ms = int(1000 / FPS)

        def push(_):
            with self.lock:
                buf_data = self.latest
            if buf_data is None:
                return True  # no frame yet

            buf = Gst.Buffer.new_allocate(None, len(buf_data), None)
            buf.fill(0, buf_data)
            # PTS/DTS omitted → appsrc timestamps automatically
            if appsrc.emit("push-buffer", buf) != Gst.FlowReturn.OK:
                return False
            return True

        GLib.timeout_add(frame_period_ms, push, None)

# ---------- Main ----------
def main():
    rclpy.init()
    node = Cam1RTSP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
