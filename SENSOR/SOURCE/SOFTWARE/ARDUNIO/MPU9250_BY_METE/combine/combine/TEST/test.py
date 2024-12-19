import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
ax.set_xlim(-10, 10)  # Adjust based on your accelerometer range
ax.set_ylim(-10, 10)
ax.set_zlim(-10, 10)

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
        return x, y, z
    except (IndexError, ValueError) as e:
        print(f"Parsing error: {e}")
        return None, None, None

try:
    # Open serial connection
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            if ser.in_waiting:
                try:
                    # Read the "DATA:" block
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line == "DATA:":
                        # Capture the next three lines for X, Y, and Z values
                        data_lines = [line]
                        for _ in range(3):
                            data_lines.append(ser.readline().decode('utf-8', errors='ignore').strip())

                        # Parse accelerometer data
                        x, y, z = parse_data(data_lines)
                        if x is not None and y is not None and z is not None:
                            x_data.append(x)
                            y_data.append(y)
                            z_data.append(z)
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