import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class SerialDataPlotter:
    def __init__(self, port, baudrate=115200):
        """Initialize the SerialDataPlotter with port and baudrate."""
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.x_data = []
        self.y_data = []

    def start_serial_connection(self):
        """Open the serial connection."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")

    def close_serial_connection(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Closed connection to {self.port}")

    def parse_serial_data(self, line):
        """Parse a line of data in the format x:<value> y:<value>."""
        try:
            line = line.decode('utf-8').strip()
            if "x:" in line and "y:" in line:
                parts = line.split()
                x = float(parts[0].split(":")[1])
                y = float(parts[1].split(":")[1])
                return x, y
        except Exception as e:
            print(f"Error parsing line: {line}, Error: {e}")
        return None, None

    def update_plot(self, frame, line, ax):
        """Update the plot with the latest data."""
        try:
            while self.ser.in_waiting > 0:
                line_data = self.ser.readline()
                x, y = self.parse_serial_data(line_data)
                if x is not None and y is not None:
                    self.x_data.append(x)
                    self.y_data.append(y)

            # Limit data to the last 100 points
            self.x_data = self.x_data[-100:]
            self.y_data = self.y_data[-100:]

            line.set_data(self.x_data, self.y_data)
            ax.relim()  # Recalculate axis limits
            ax.autoscale_view()  # Update view limits
        except Exception as e:
            print(f"Error during update: {e}")
        return line,


def main():
    # Configuration
    port = 'COM6'  # Change to your serial port
    baudrate = 115200

    # Initialize SerialDataPlotter and start serial connection
    plotter = SerialDataPlotter(port, baudrate)
    plotter.start_serial_connection()

    # Matplotlib setup
    plt.style.use('default')
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'o-', label='Data Points')

    # Axes settings
    ax.set_xlim(-0.2, 0.2)
    ax.set_ylim(1.4, 1.5)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()

    # Animation setup
    ani = FuncAnimation(
        fig, plotter.update_plot, fargs=(line, ax), interval=100
    )

    # Display the plot
    plt.show()

    # Close the serial connection after plotting
    plotter.close_serial_connection()


if __name__ == "__main__":
    main()
