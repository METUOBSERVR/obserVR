#!/usr/bin/env python3
"""
tcp_pose_sender_le_f64.py
ROS 2 ? TCP bridge (little-endian, 40 B) with
 double timestamp
 live INA226 voltage (fallback = 11.8 V)
 10-second debug prints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import socket, struct, time, threading

# ------------------ USER SETTINGS ------------------------------------------
TCP_HOST       = "192.168.0.101"#"172.20.10.8"
IMU_PORT       = 5005
CAM_PORT       = 5006
FUSION_PORT    = 5007
SEND_HZ        = 100
DBG_PERIOD_S   = 10.0          # print once per stream every N seconds
# INA226 / INA219
INA_ADDR       = 0x40
REG_BUS_VOLT   = 0x02
LSB_VOLT       = 1.25e-3       # 1.25 mV per bit
# ---------------------------------------------------------------------------

PKT = struct.Struct('<dffffffff')   # double ts + 8float32 = 40 B


# ------------- voltage helper ----------------------------------------------
try:
    from smbus2 import SMBus
    _bus = SMBus(1)
except (ImportError, FileNotFoundError):
    _bus = None                    # no IC or smbus2 not installed


def read_voltage_v() -> float:
    """Return bus voltage in volts; fall back to constant on error."""
    if _bus is None:
        return 11.8
    try:
        raw = _bus.read_word_data(INA_ADDR, REG_BUS_VOLT)
        raw = ((raw << 8) & 0xFF00) | (raw >> 8)     # byte-swap
        return raw * LSB_VOLT
    except OSError:
        return 11.8
# ---------------------------------------------------------------------------


class PoseTCPSender(Node):
    def __init__(self):
        super().__init__('tcp_pose_sender_le_f64')

        self._imu_pose = self._cam_pose = self._fusion_pose = None
        self._lock = threading.Lock()

        # ROS subscriptions --------------------------------------------------
        self.create_subscription(
            PoseStamped, '/IMUpose',
            lambda m: self._set('_imu_pose', m.pose), 10)

        self.create_subscription(
            PoseStamped, '/CAMpose',
            lambda m: self._set('_cam_pose', m.pose), 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/ov_msckf/poseimu',
            lambda m: self._set('_fusion_pose', m.pose.pose), 10)   # ? fixed typo

        # one worker per stream ---------------------------------------------
        threading.Thread(target=self._worker,
                         args=('imu',    IMU_PORT,    '_imu_pose'),
                         daemon=True).start()
        threading.Thread(target=self._worker,
                         args=('cam',    CAM_PORT,    '_cam_pose'),
                         daemon=True).start()
        threading.Thread(target=self._worker,
                         args=('fusion', FUSION_PORT, '_fusion_pose'),
                         daemon=True).start()

    # ------------- helpers -------------------------------------------------
    def _set(self, attr, msg):
        with self._lock:
            setattr(self, attr, msg)

    def _packet(self, pose) -> bytes:
        p, q = pose.position, pose.orientation
        return PKT.pack(
            time.time(),            # float64 seconds
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            read_voltage_v()        # live voltage
        )

    def _connect(self, port: int):
        while rclpy.ok():
            try:
                s = socket.create_connection((TCP_HOST, port))
                self.get_logger().info(f'Connected to {TCP_HOST}:{port}')
                return s
            except OSError as e:
                self.get_logger().warn(f'{port}: {e}; retrying in 1 s')
                time.sleep(1)

    # ------------- worker --------------------------------------------------
    def _worker(self, tag: str, port: int, pose_attr: str):
        period   = 1.0 / SEND_HZ
        next_dbg = time.time() + DBG_PERIOD_S
        sock     = None

        while rclpy.ok():
            if sock is None:
                sock = self._connect(port)

            with self._lock:
                pose = getattr(self, pose_attr)

            if pose:
                try:
                    sock.sendall(self._packet(pose))
                except (BrokenPipeError, OSError):
                    sock.close(); sock = None

            # periodic debug print
            now = time.time()
            if now >= next_dbg:
                next_dbg = now + DBG_PERIOD_S
                if pose:
                    self.get_logger().info(
                        f'[{tag.upper():6}] ts={now:.1f}  '
                        f'pos=({pose.position.x:+.3f},'
                        f'{pose.position.y:+.3f},'
                        f'{pose.position.z:+.3f})  '
                        f'volt={read_voltage_v():.2f} V'
                    )
                else:
                    self.get_logger().info(f'[{tag.upper():6}] no pose yet')

            time.sleep(period)


# ------------------------------ main ---------------------------------------
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
