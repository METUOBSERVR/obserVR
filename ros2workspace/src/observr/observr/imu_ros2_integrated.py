# NECESSARY LIBRARIES
# IMU
import adafruit_bno055
import busio

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# SYSTEM
import time


class BNO055Observer(Node):
    CONFIG_MODE = 0x00
    NDOF_MODE = 0x0C

    def __init__(self):
        super().__init__('imu_publisher_node')

        # Declare ROS parameters
        self.declare_parameter('sample_amount', 1000)
        self.declare_parameter('publish_period_sec', 0.01)
        # Initialize the I2C bus and the BNO055 sensor
        self.i2c = busio.I2C(scl=3, sda=2)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.sample_amount = self.get_parameter('sample_amount').get_parameter_value().integer_value
        self.publish_period = self.get_parameter('publish_period_sec').get_parameter_value().double_value
        self.sensor.mode = self.CONFIG_MODE
        time.sleep(0.5)
        self.sensor.remap = 0x18
        time.sleep(0.5)
        self.sensor.mode = self.NDOF_MODE

        # Biases
        self.linearAccelBiasX = 0.0
        self.linearAccelBiasY = 0.0
        self.linearAccelBiasZ = 0.0
        self.angularVelBiasX = 0.0
        self.angularVelBiasY = 0.0
        self.angularVelBiasZ = 0.0

        # State
        self.locX = self.locY = self.locZ = 0.0
        self.velX = self.velY = self.velZ = 0.0
        self.last_time = time.time()

        self.get_logger().info("IMU node initialized")
        self.get_logger().info("IMU calibration status: " + str(self.sensor.calibration_status))

        self.wait_for_built_in_calibration()
        self.get_logger().info("IMU is calibrated")

      #  self.return_imu_bias_calibration()
      #  time.sleep(10)
      #  self.wait_for_imu_bias_calibration()
      #  self.return_imu_bias_calibration()
      #  self.get_logger().info("IMU bias calibration completed")

        # ROS2 publishers
        self.publisher_ = self.create_publisher(Imu, '/imu0', 5)
        self.publisherRAW_ = self.create_publisher(Imu, '/imu_raw', 5)
        self.publisherPOS_ = self.create_publisher(PoseStamped, '/IMUpose', 10)
        self.timer = self.create_timer(self.publish_period, self.publish_imu_data)

    def wait_for_built_in_calibration(self):
        while self.sensor.calibration_status != (3,3,3,3):
            time.sleep(2)
            self.get_logger().info("IMU calibration status: " + str(self.sensor.calibration_status))

    def return_imu_bias_calibration(self):
        self.get_logger().info("IMU bias calibration:")
        self.get_logger().info(f"Linear Accel Bias: X={self.linearAccelBiasX:.2f}, Y={self.linearAccelBiasY:.2f}, Z={self.linearAccelBiasZ:.2f}")
        self.get_logger().info(f"Angular Vel Bias: X={self.angularVelBiasX:.2f}, Y={self.angularVelBiasY:.2f}, Z={self.angularVelBiasZ:.2f}")

    def wait_for_imu_bias_calibration(self):
        self.get_logger().info("Collecting samples for IMU bias calibration...")
        count = 0
        for _ in range(self.sample_amount):
            time.sleep(0.01)
            accel = self.sensor.linear_acceleration
            gyro = self.sensor.gyro
            if accel is None or gyro is None:
                continue
            self.linearAccelBiasX += accel[0]
            self.linearAccelBiasY += accel[1]
            self.linearAccelBiasZ += accel[2]
            self.angularVelBiasX += gyro[0]
            self.angularVelBiasY += gyro[1]
            self.angularVelBiasZ += gyro[2]
            count += 1

        if count > 0:
            self.linearAccelBiasX /= count
            self.linearAccelBiasY /= count
            self.linearAccelBiasZ /= count
            self.angularVelBiasX /= count
            self.angularVelBiasY /= count
            self.angularVelBiasZ /= count

    def publish_imu_data(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/imu_data"

        msgRaw = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "/imu_data_raw"

        # Orientation
        quat = self.sensor.quaternion
        if quat is not None:
            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]

            msgRaw.orientation.x = quat[0]
            msgRaw.orientation.y = quat[1]
            msgRaw.orientation.z = quat[2]
            msgRaw.orientation.w = quat[3]

        msg.orientation_covariance = [0.0159, 0.0, 0.0,
                                      0.0, 0.0159, 0.0,
                                      0.0, 0.0, 0.0159]
        msgRaw.orientation_covariance = [0.0159, 0.0, 0.0,
                                        0.0, 0.0159, 0.0,
                                        0.0, 0.0, 0.0159]

        # Angular velocity
        gyro = self.sensor.gyro
        if gyro is not None:
            msg.angular_velocity.x = gyro[0] - self.angularVelBiasX
            msg.angular_velocity.y = gyro[1] - self.angularVelBiasY
            msg.angular_velocity.z = gyro[2] - self.angularVelBiasZ

            msgRaw.angular_velocity.x = gyro[0] - self.angularVelBiasX
            msgRaw.angular_velocity.y = gyro[1] - self.angularVelBiasY
            msgRaw.angular_velocity.z = gyro[2] - self.angularVelBiasZ

        msg.angular_velocity_covariance = [0.04, 0.0, 0.0,
                                           0.0, 0.04, 0.0,
                                           0.0, 0.0, 0.04]

        msgRaw.angular_velocity_covariance = [0.04, 0.0, 0.0,
                                              0.0, 0.04, 0.0,
                                              0.0, 0.0, 0.04]

        #Accelerometer
        accelerometer = self.sensor.acceleration
        if accelerometer is not None:
            amx = accelerometer[0]
            amy = accelerometer[1]
            amz = accelerometer[2]

            msgRaw.linear_acceleration.x = amx
            msgRaw.linear_acceleration.y = amy
            msgRaw.linear_acceleration.z = amz

        msgRaw.linear_acceleration_covariance = [0.017, 0.0, 0.0,
                                                 0.0, 0.017, 0.0,
                                                 0.0, 0.0, 0.017]

        # Linear acceleration
        accel = self.sensor.linear_acceleration
        if accel is not None:
            ax = accel[0] - self.linearAccelBiasX
            ay = accel[1] - self.linearAccelBiasY
            az = accel[2] - self.linearAccelBiasZ

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            # Velocity and position estimation using trapezoidal integration
            self.velX += ax * dt
            self.velY += ay * dt
            self.velZ += az * dt
            self.locX += self.velX * dt
            self.locY += self.velY * dt
            self.locZ += self.velZ * dt

        msg.linear_acceleration_covariance = [0.017, 0.0, 0.0,
                                              0.0, 0.017, 0.0,
                                              0.0, 0.0, 0.017]

        # PoseStamped message
        msg2 = PoseStamped()
        msg2.header = Header()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.header.frame_id = "/imu_pose_raw"
        msg2.pose.position.x = self.locX
        msg2.pose.position.y = self.locY
        msg2.pose.position.z = self.locZ
        if quat is not None:
            msg2.pose.orientation.x = quat[0]
            msg2.pose.orientation.y = quat[1]
            msg2.pose.orientation.z = quat[2]
            msg2.pose.orientation.w = quat[3]

        # Publish
        self.publisher_.publish(msg)
        self.publisherRAW_.publish(msgRaw)
        self.publisherPOS_.publish(msg2)
       # self.get_logger().info('Publishing IMU data...')


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = BNO055Observer()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
