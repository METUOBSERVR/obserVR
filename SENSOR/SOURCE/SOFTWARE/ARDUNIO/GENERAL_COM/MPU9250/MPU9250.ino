#include <Wire.h>
#include <stdint.h>

// GENERAL CONSTANT AND PARAMETER
// ---
#define MPU_DEVICE_ADDRESS 0x68
// ---

// GENERAL MPU9250
// ---
#define WHO_I_AM_ADDRESS 0x75
// ---


// GENERAL I2C FUNCTION 
// ---
void write_mpu9250(uint8_t reg_addr, uint8_t data);
uint8_t read_mpu9250(uint8_t reg_addr);
// ---

// GENERAL MPU9250 FUNCTION
// ---
uint8_t getID();
// ---


void setup() {
    // INITIALIZING SERIAL
    Serial.begin(115200);
    // INITIALIZING I2C
    Wire.setClock(100000);      // clock frequency in hertz 
    Wire.begin();               // initializing as master

    // Reset the MPU9250
    write_mpu9250(0x6B, 0x00); // Write 0x00 to the PWR_MGMT_1 register to wake up the MPU9250
    delay(100);

    Serial.println("MPU9250 initialized.");
}

void loop() {
    // Read WHO_AM_I register (0x75) for MPU9250 identification
    uint8_t who_am_i = getID();
    Serial.print("WHO_AM_I register value: ");
    Serial.println(who_am_i, HEX);
    Serial.print("\n");
    delay(500);
}

void write_mpu9250(uint8_t reg_addr, uint8_t data) {
    Wire.beginTransmission(MPU_DEVICE_ADDRESS);
    Wire.write(reg_addr);
    Wire.write(data);
    Wire.endTransmission(true);
}

uint8_t read_mpu9250(uint8_t reg_addr) {
    Wire.beginTransmission(MPU_DEVICE_ADDRESS);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(MPU_DEVICE_ADDRESS, 1, true);
    return Wire.read();
}

uint8_t getID()
{
    return read_mpu9250(WHO_I_AM_ADDRESS);
};



