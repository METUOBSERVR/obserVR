import serial

# Set up the serial connection (adjust 'COMx' to your specific port)
ser = serial.Serial('COM6', 115200, timeout=1)

while True:
    # Read the full package (this should contain 3 pieces of data separated by '\n')
    package = ser.read_until(b'\n')  # Read until newline byte
    
    # Split the data by newline and process each part
    data_parts = package.decode('utf-8').strip('\n').strip("LOG:").split('-')
    sign = [1,1,1]
    if len(data_parts) == 3:
        # Convert each part to an integer
        for i in range(3):
            sign[i] = 1
            if (len(data_parts[i]) >= 4 ):
                data_parts[i] - 0xFFFFFFFF
                              
                
        #data1 = int(data_parts[0],16) * sign[0] * 4 / (2**16)
        #data2 = int(data_parts[1],16) * sign[1] * 4 / (2**16)
        #data3 = int(data_parts[2],16) * sign[2] * 4 / (2**16)  
        print(f"Data 1: {data_parts[0]}")
        print(f"Data 2: {data_parts[1]}")
        print(f"Data 3: {data_parts[2]}")
    else:
        print("Invalid package format: Expected 3 pieces of data")
