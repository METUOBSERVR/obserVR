#!/usr/bin/env python3
#
# TCP sender for /imu_pose_raw  (PORT 5005)
#            and /cam_pose_raw  (PORT 5006)
#
# Packet (little-endian, 40 bytes):
#   struct.pack('<d f f f f f f f f',
#               unix_time,
#               x, y, z,
#               qx, qy, qz, qw,
#               voltage)
#
# Replace get_voltage_placeholder() with real INA226 / ADC code.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket, struct, time, threading

# ---------------------------- USER SETTINGS ---------------------------------
TCP_HOST   = '10.180.130.167'   # <- Windows PC IP
IMU_PORT   = 5005
CAM_PORT   = 5006
SEND_HZ    = 100              # independent send rate for each stream
# ---------------------------------------------------------------------------

def get_voltage_placeholder() -> float:
    # TOD: implement actual measurement
    return 11.5

class PoseTCPSender(Node):
    def __init__(self):
        super().__init__('tcp_pose_sender')

        # shared state
        self._imu_pose = None
        self._cam_pose = None
        self._lock     = threading.Lock()

        # ROS subscriptions ---------------------------------------------------
        self.create_subscription(PoseStamped, '/IMUpose', self._imu_cb,  10)
        self.create_subscription(PoseStamped, '/CAMpose', self._cam_cb, 10)

        # socket handles
        self._imu_sock = None
        self._cam_sock = None

        # worker threads ------------------------------------------------------
        threading.Thread(target=self._worker,
                         args=('imu', IMU_PORT, SEND_HZ),
                         daemon=True).start()
        threading.Thread(target=self._worker,
                         args=('cam', CAM_PORT, SEND_HZ),
                         daemon=True).start()

    # ---------------- ROS callbacks -----------------
    def _imu_cb(self, msg: PoseStamped):
        with self._lock:
            self._imu_pose = msg.pose

    def _cam_cb(self, msg: PoseStamped):
        with self._lock:
            self._cam_pose = msg.pose

    # ---------------- worker loop ------------------
    def _worker(self, which: str, port: int, hz: float):
        sock_attr  = f'_{which}_sock'
        pose_attr  = f'_{which}_pose'
        period     = 1.0 / hz

        while rclpy.ok():
            sock = getattr(self, sock_attr)
            if sock is None:
                sock = self._connect(port)
                setattr(self, sock_attr, sock)

            with self._lock:
                pose = getattr(self, pose_attr)

            if pose:
                pkt = self._build_packet(pose)
                try:
                    sock.sendall(pkt)
                except (BrokenPipeError, OSError):
                    sock.close()
                    setattr(self, sock_attr, None)

            time.sleep(period)

    # ------------- packet builder ------------------
    def _build_packet(self, pose: PoseStamped) -> bytes:
        p = pose.position
        q = pose.orientation
        return struct.pack('<dffffffff',
                           time.time(),
                           p.x, p.y, p.z,
                           q.x, q.y, q.z, q.w,
                           get_voltage_placeholder())

    # ------------- connection helper ---------------
    def _connect(self, port: int) -> socket.socket:
        while rclpy.ok():
            try:
                s = socket.create_connection((TCP_HOST, port))
                self.get_logger().info(f'Connected to {TCP_HOST}:{port}')
                return s
            except OSError as e:
                self.get_logger().warn(f'{port}: {e}; retrying in 1 s')
                time.sleep(1)

# --------------------------- main ---------------------------
def main():
    rclpy.init()
    node = PoseTCPSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
