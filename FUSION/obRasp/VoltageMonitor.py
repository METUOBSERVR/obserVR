#!/usr/bin/env python3
from smbus2 import SMBus

# INA226 default I2C address (can be different based on A0/A1 pin config)
INA226_ADDRESS = 0x40

# Register addresses
REG_BUS_VOLTAGE = 0x02  # Bus voltage register

# LSB size for bus voltage = 1.25 mV (per datasheet)
BUS_VOLTAGE_LSB = 1.25e-3  # in volts

def read_bus_voltage(bus, address):
    # Read 2 bytes from the Bus Voltage register
    raw = bus.read_word_data(address, REG_BUS_VOLTAGE)

    # Swap bytes because INA226 uses little-endian format
    raw = ((raw << 8) & 0xFF00) + (raw >> 8)

    voltage = raw * BUS_VOLTAGE_LSB  # Convert to volts
    return voltage

if __name__ == "__main__":
    bus_num = 1  # Typically I2C-1 on Raspberry Pi
    with SMBus(bus_num) as bus:
        voltage = read_bus_voltage(bus, INA226_ADDRESS)
        print(f"Bus Voltage: {voltage:.3f} V")

