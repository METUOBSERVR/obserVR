#include <Wire.h>
#include <stdint.h>
#include <math.h>

// GENERAL CONSTANT AND PARAMETER
// ---
#define MPU_DEVICE_ADDRESS 0x68
#define MPU_WIAM_DATA 0x70
// ---

// GENERAL REGISTER MAP FOR MPU9250
// ---
#define INTERRUPT_STATUS_ADDRESS 0x3A
#define WHO_I_AM_ADDRESS 0x75
#define ACCEL_XH_ADDRESS 0x3B
#define ACCEL_XL_ADDRESS 0x3C
#define ACCEL_YH_ADDRESS 0x3D
#define ACCEL_YL_ADDRESS 0x3E
#define ACCEL_ZH_ADDRESS 0x3F
#define ACCEL_ZL_ADDRESS 0x40
#define ACCEL_XH_OFFSET_ADDRES 0x77
#define ACCEL_XL_OFFSET_ADDRES 0x78
#define ACCEL_YH_OFFSET_ADDRES 0x7A
#define ACCEL_YL_OFFSET_ADDRES 0x7B
#define ACCEL_ZH_OFFSET_ADDRES 0x7D
#define ACCEL_ZL_OFFSET_ADDRES 0x7E

// ---

// STATIC MAIN VARIABLES 
// ---
static uint8_t  acceleration [3][2];
static uint16_t accelerationX;
static uint16_t accelerationY;
static uint16_t accelerationZ;
static int16_t  signed_accelerationX;
static int16_t  signed_accelerationY;
static int16_t  signed_accelerationZ;
static float    float_signed_accelerationX;
static float    float_signed_accelerationY;
static float    float_signed_accelerationZ;
static float    x_velocity = 0;
static float    y_velocity = 0;
static float    z_velocity = 0;
static float    x_location = 0;
static float    y_location = 0;
static float    z_location = 0;
static int      loc_log = 0;
/*
  acceleration [[xMSB][xLSB]
                [yMSB][yLSB]
                [ZMSB][zLSB]]
*/

// ---


// GENERAL I2C FUNCTION 
// ---
void write_mpu9250(uint8_t reg_addr, uint8_t data);
uint8_t read_mpu9250(uint8_t reg_addr);

// ---

// GENERAL MPU9250 FUNCTION
// ---
uint8_t getID();
void getAcceleration();
void updateAcceleration();
void printAcceleration();
void getUpdatePrintAcceleration();
void calibrationAcceleration();
void updateAccelerationXoffset();
void updateAccelerationYoffset();
void updateAccelerationZoffset();
void RawToFloatConversionAcceleration();
bool RawDataReady();
void log_acceleration();
// ---


void setup() 
{
    // INITIALIZING SERIAL
    Serial.begin(115200);
    // INITIALIZING I2C
    Wire.setClock(400000);      // clock frequency in hertz 
    Wire.begin();               // initializing as master

    // Reset the MPU9250
    write_mpu9250(0x6B, 0x00); // Write 0x00 to the PWR_MGMT_1 register to wake up the MPU9250
    delay(100);
    // Read WHO_AM_I register (0x75) for MPU9250 identification
    uint8_t who_am_i = getID();
    if (who_am_i == MPU_WIAM_DATA)
    {
        Serial.println("Device address is detected correctly. I2C works properly...\n");
    };
    delay(100);
    // First read
    calibrationAcceleration(); 

}


void loop() 
{
    if (RawDataReady())
    {
        
        getUpdatePrintAcceleration();
        RawToFloatConversionAcceleration();
        log_acceleration();
        if (fabs(float_signed_accelerationX) < 0.2)
        {
            float_signed_accelerationX = 0;
        };
        if (fabs(float_signed_accelerationY) < 0.2)
        {
            float_signed_accelerationY = 0;
        };
        if (fabs(float_signed_accelerationZ) < 0.2)
        {
            float_signed_accelerationZ = 0;
        };
        x_velocity  += float_signed_accelerationX * 0.250;
        y_velocity  += float_signed_accelerationY * 0.250;
        z_velocity  += float_signed_accelerationZ * 0.250;
        if (fabs(x_velocity) < 0.05)
        {
            x_velocity = 0;
        };
        if (fabs(y_velocity) < 0.05)
        {
            y_velocity = 0;
        };
        if (fabs(z_velocity) < 0.05)
        {
            z_velocity = 0;
        };
        x_location  += x_velocity * 0.250;
        y_location  += y_velocity * 0.250;
        z_location  += z_velocity * 0.250;
        delay(200);
        loc_log++;
    }
    if(false)
    {
        loc_log = 0;
        Serial.println("X:\n");
        Serial.println(x_location);
        Serial.println("Y:\n");
        Serial.println(y_location);
        Serial.println("Z:\n");
        Serial.println(z_location);
    };
}

void write_mpu9250(uint8_t reg_addr, uint8_t data) {
    Wire.beginTransmission(MPU_DEVICE_ADDRESS);
    Wire.write(reg_addr);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t read_mpu9250(uint8_t reg_addr) {
    Wire.beginTransmission(MPU_DEVICE_ADDRESS);
    Wire.write(reg_addr);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_DEVICE_ADDRESS, 1);
    if (Wire.available()) return Wire.read();
    return 0x00;
}

uint8_t getID()
{
    return read_mpu9250(WHO_I_AM_ADDRESS);
};

void getAcceleration()
{
    acceleration[0][0]  = read_mpu9250(ACCEL_XH_ADDRESS);
    acceleration[0][1]  = read_mpu9250(ACCEL_XL_ADDRESS);
    acceleration[1][0]  = read_mpu9250(ACCEL_YH_ADDRESS);
    acceleration[1][1]  = read_mpu9250(ACCEL_YL_ADDRESS);
    acceleration[2][0]  = read_mpu9250(ACCEL_ZH_ADDRESS);
    acceleration[2][1]  = read_mpu9250(ACCEL_ZL_ADDRESS);
};


void updateAcceleration()
{
    accelerationX         = ((uint16_t)acceleration[0][0]) << 8 | acceleration[0][1]; 
    accelerationY         = ((uint16_t)acceleration[1][0]) << 8 | acceleration[1][1]; 
    accelerationZ         = ((uint16_t)acceleration[2][0]) << 8 | acceleration[2][1]; 
    signed_accelerationX  = ((int16_t)accelerationX);
    signed_accelerationY  = ((int16_t)accelerationY);
    signed_accelerationZ  = ((int16_t)accelerationZ);
};

void printAcceleration()
{
    Serial.print("Acceleration:\nx->");
    Serial.println(signed_accelerationX);
    Serial.print("y->");
    Serial.println(signed_accelerationY);
    Serial.print("z->");
    Serial.println(signed_accelerationZ);
};


void getUpdatePrintAcceleration()
{
    getAcceleration();
    updateAcceleration();
    //printAcceleration();
};

void updateAccelerationXoffset(int data)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;
    lsb = (uint8_t)data; 
    write_mpu9250(ACCEL_XL_OFFSET_ADDRES, lsb);
    data /=  256;
    msb = (uint8_t)data;
    write_mpu9250(ACCEL_XH_OFFSET_ADDRES, msb);
};

void updateAccelerationYoffset(int data)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;
    lsb = (uint8_t) data; 
    write_mpu9250(ACCEL_YL_OFFSET_ADDRES, lsb);
    data /=  256;
    msb = (uint8_t) data;
    write_mpu9250(ACCEL_YH_OFFSET_ADDRES, msb);
};

void updateAccelerationZoffset(int data)
{
    uint8_t msb = 0;
    uint8_t lsb = 0;
    lsb = (uint8_t) data;  
    write_mpu9250(ACCEL_ZL_OFFSET_ADDRES, lsb);
    data /=  256;
    msb = (uint8_t) data;
    write_mpu9250(ACCEL_ZH_OFFSET_ADDRES, msb);

};



void calibrationAcceleration()
{

    int X = 0 ,Y = 0,Z = 0;
    int Xb = 0 ,Yb = 0,Zb = 0;
    Xb =  read_mpu9250(ACCEL_XH_OFFSET_ADDRES) * 256 +  read_mpu9250(ACCEL_XL_OFFSET_ADDRES);
    Yb =  read_mpu9250(ACCEL_YH_OFFSET_ADDRES) * 256 +  read_mpu9250(ACCEL_YL_OFFSET_ADDRES);
    Zb =  read_mpu9250(ACCEL_ZH_OFFSET_ADDRES) * 256 +  read_mpu9250(ACCEL_ZL_OFFSET_ADDRES);

    for (int i = 0; i < 8; i++)
    {
        getAcceleration();
        updateAcceleration();
        delay(125);       // change here with data ready?
        X = X + (int)signed_accelerationX;
        Y = Y + (int)signed_accelerationY;
        Z = Z + ((int)signed_accelerationZ) - 16384 ; // 1g = 16384 
    };
    X = X >> 6;
    Y = Y >> 6;
    Z = Z >> 6;
    X = -X + Xb;
    Y = -Y + Yb;
    Z = -Z + Zb;
    updateAccelerationXoffset(X);
    updateAccelerationYoffset(Y);
    updateAccelerationZoffset(Z - 16384/8);


}

void RawToFloatConversionAcceleration()
{
    float_signed_accelerationX = signed_accelerationX;
    float_signed_accelerationY = signed_accelerationY;
    float_signed_accelerationZ = signed_accelerationZ;
    float_signed_accelerationX = float_signed_accelerationX * (4) /65536;
    float_signed_accelerationY = float_signed_accelerationY * (4) /65536;
    float_signed_accelerationZ = float_signed_accelerationZ * (4) /65536;
};

bool RawDataReady()
{
    uint8_t is_ready;
    is_ready = read_mpu9250(INTERRUPT_STATUS_ADDRESS);
    is_ready = is_ready & 0x01;
    return (is_ready > 0) ? true : false;
};

void log_acceleration()
{
    Serial.print("LOG:");
    Serial.print(signed_accelerationX,HEX);
    Serial.print("-");
    Serial.print(signed_accelerationY,HEX);
    Serial.print("-");
    Serial.print(signed_accelerationZ,HEX);
    Serial.print("\n");
};




