import serial
import matplotlib.pyplot as plt
import time

# Initialize global variables
x_data = []
y_data = []
z_data = []

# Serial port configuration
ser = serial.Serial(
    port='COM6',  # Change this to your COM port
    baudrate=115200,
    timeout=1
)

def parse_data(data):
    """
    Parses the incoming data from the serial port to extract IMU values.
    """
    imu_data = {
        "Location:": {"X": 0.0, "Y": 0.0, "Z": 0.0},
        "Filtered Location:": {"X": 0.0, "Y": 0.0, "Z": 0.0},
        "Target Location:": {"X": 0.0, "Y": 0.0, "Z": 0.0}
    }

    current_section = None
    for line in data.splitlines():
        line = line.strip()
        if not line:
            continue  # Skip empty lines
        if line in imu_data:
            current_section = line
            print(f"Detected section: {current_section}")  # Debugging line
        elif current_section:
            try:
                if line.startswith("X:"):
                    imu_data[current_section]['X'] = float(line.split()[1])
                elif line.startswith("Y:"):
                    imu_data[current_section]['Y'] = float(line.split()[1])
                elif line.startswith("Z:"):
                    imu_data[current_section]['Z'] = float(line.split()[1])
            except (IndexError, ValueError) as e:
                print(f"Error parsing line '{line}': {e}")  # Debugging line

    return imu_data


def read_serial_data():
    """
    Reads data from the serial port and parses it.
    """
    if ser.in_waiting > 0:
        raw_data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
        print("Raw data received:", raw_data)  # Debugging line
        return parse_data(raw_data)
    return None

def update_plot():
    """
    Continuously update the plot with new data from the serial port.
    """
    global x_data, y_data, z_data

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(0, 100)  # Adjust as needed
    ax.set_ylim(-10, 10)  # Adjust as needed

    x_line, = ax.plot([], [], label="X")
    y_line, = ax.plot([], [], label="Y")
    z_line, = ax.plot([], [], label="Z")

    plt.legend()
    plt.xlabel("Time")
    plt.ylabel("Value")

    while True:
        imu_data = read_serial_data()
        if imu_data:
            x_data.append(imu_data["Location:"]["X"])
            y_data.append(imu_data["Location:"]["Y"])
            z_data.append(imu_data["Location:"]["Z"])

            # Keep the plot within the last 100 data points
            if len(x_data) > 100:
                x_data = x_data[-100:]
                y_data = y_data[-100:]
                z_data = z_data[-100:]

            x_line.set_data(range(len(x_data)), x_data)
            y_line.set_data(range(len(y_data)), y_data)
            z_line.set_data(range(len(z_data)), z_data)

            ax.set_xlim(0, len(x_data))

            plt.pause(0.01)

if __name__ == "__main__":
    try:
        print("Starting to read and plot data...")
        update_plot()
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()
        print("Serial port closed.")
