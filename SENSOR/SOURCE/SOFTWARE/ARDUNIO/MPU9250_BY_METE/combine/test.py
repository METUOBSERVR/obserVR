import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time

# Madgwick Filter Implementation (Simple version)
class MadgwickFilter:
    def __init__(self, beta=0.1):
        self.beta = beta  # Filter gain
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0  # Quaternion values

    def update(self, ax, ay, az, gx, gy, gz, dt):
        # Auxiliary variables
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        ax, ay, az = ax/norm, ay/norm, az/norm  # Normalize accelerometer measurement

        # Gradient descent algorithm corrective step
        s0 = -2.0 * (q2 * gx - q1 * gy + q3 * gz)
        s1 = 2.0 * (q1 * gx + q2 * gy + q0 * gz)
        s2 = 2.0 * (q0 * gx - q3 * gy - q1 * gz)
        s3 = 2.0 * (q3 * gx + q0 * gy + q2 * gz)

        # Normalize the gradient to make it unit length
        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        s0, s1, s2, s3 = s0 / norm, s1 / norm, s2 / norm, s3 / norm

        # Apply the gradient descent correction to the quaternion
        q0 += s0 * dt * 0.5
        q1 += s1 * dt * 0.5
        q2 += s2 * dt * 0.5
        q3 += s3 * dt * 0.5

        # Normalize the quaternion
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q0, self.q1, self.q2, self.q3 = q0 / norm, q1 / norm, q2 / norm, q3 / norm

    def get_euler_angles(self):
        # Compute pitch, roll, yaw from the quaternion
        pitch = math.atan2(2.0*(self.q0*self.q1 + self.q2*self.q3), 1.0 - 2.0*(self.q1*self.q1 + self.q2*self.q2))
        roll = math.asin(2.0*(self.q0*self.q2 - self.q3*self.q1))
        yaw = math.atan2(2.0*(self.q0*self.q3 + self.q1*self.q2), 1.0 - 2.0*(self.q2*self.q2 + self.q3*self.q3))

        # Convert from radians to degrees
        pitch, roll, yaw = math.degrees(pitch), math.degrees(roll), math.degrees(yaw)
        return pitch, roll, yaw

# Serial Configuration
SERIAL_PORT = 'COM6'  # Replace with your Arduino's serial port
BAUD_RATE = 115200      # Match the baud rate of your Arduino
UPDATE_INTERVAL = 10  # Number of captures before updating the plot

# Initialize variables for plotting
x_data, y_data, z_data = [], [], []
capture_count = 0

# Initialize the plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot formatting
ax.set_title("Real-Time Accelerometer Data")
ax.set_xlabel("X-Axis (g)")
ax.set_ylabel("Y-Axis (g)")
ax.set_zlabel("Z-Axis (g)")
ax.set_xlim(-1, 1)  # Adjust based on your accelerometer range
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

# Initialize the Madgwick filter
madgwick_filter = MadgwickFilter(beta=0.1)

def update_plot():
    """Update Matplotlib 3D plot with new data."""
    ax.clear()
    ax.set_title("Real-Time Accelerometer Data")
    ax.set_xlabel("X-Axis (g)")
    ax.set_ylabel("Y-Axis (g)")
    ax.set_zlabel("Z-Axis (g)")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.scatter(x_data, y_data, z_data, c='b', marker='o')  # Scatter plot for real-time data
    plt.draw()
    plt.pause(0.01)

def parse_data(data_lines):
    """Parse the DATA section from serial output."""
    try:
        x = float(data_lines[1])
        y = float(data_lines[2])
        z = float(data_lines[3])
        gx = float(data_lines[4])  # Gyroscope X
        gy = float(data_lines[5])  # Gyroscope Y
        gz = float(data_lines[6])  # Gyroscope Z
        return x, y, z, gx, gy, gz
    except (IndexError, ValueError) as e:
        print(f"Parsing error: {e}")
        return None, None, None, None, None, None

try:
    # Open serial connection
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        prev_time = time.time()
        while True:
            if ser.in_waiting:
                try:
                    # Read the "DATA:" block
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line == "DATA:":
                        # Capture the next three lines for X, Y, and Z values
                        data_lines = [line]
                        for _ in range(6):
                            data_lines.append(ser.readline().decode('utf-8', errors='ignore').strip())

                        # Parse accelerometer and gyroscope data
                        x, y, z, gx, gy, gz = parse_data(data_lines)
                        if x is not None and y is not None and z is not None:
                            # Apply Madgwick filter with accelerometer and gyroscope data
                            current_time = time.time()
                            dt = current_time - prev_time
                            prev_time = current_time
                            madgwick_filter.update(x, y, z, gx, gy, gz, dt)
                            pitch, roll, yaw = madgwick_filter.get_euler_angles()

                            # Store pitch, roll, yaw for plotting
                            x_data.append(pitch)
                            y_data.append(roll)
                            z_data.append(yaw)
                            capture_count += 1

                        # Update plot every UPDATE_INTERVAL captures
                        if capture_count % UPDATE_INTERVAL == 0:
                            update_plot()

                except Exception as e:
                    print(f"Error: {e}")

except KeyboardInterrupt:
    print("Program terminated by user.")
except Exception as e:
    print(f"Error: {e}")
finally:
    plt.ioff()
    plt.show()
