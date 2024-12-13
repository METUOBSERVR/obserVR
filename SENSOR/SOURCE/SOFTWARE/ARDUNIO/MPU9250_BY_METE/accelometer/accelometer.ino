#include <Wire.h>
#include <stdint.h>
#include <math.h>


#define MPU9250_DEVICE_ADDRESS 0x68

static uint8_t  RawAcceleration[3][2];
static uint16_t Raw16Acceleration[3];
static int16_t  SRaw16Acceleration[3];
static float    FloatAcceleration[3];
static float    FloatVelocity[3];
static float    FloatLocation[3];
static float    BackFloatAcceleration[3];
static int print = 0;

volatile  bool check = false;

#define MPU9250_WHOIAM_REGISTER_ADDRESS 0x75
#define MPU9250_INTERRUPT_CONFIG_ADDRESS 0x37
#define MPU9250_INTERRUPT_ENABLE_ADDRESS 0x38
#define MPU9250_INTERRUPT_STATUS_ADDRESS 0x3A
#define MPU9250_POWER_MGMNT_1_ADDRESS 0x6B
#define MPU9250_POWER_MGMNT_2_ADDRESS 0x6C
#define MPU9250_ACCELERATION_XHRAW_ADDRESS 0x3B
#define MPU9250_ACCELERATION_XLRAW_ADDRESS 0x3C
#define MPU9250_ACCELERATION_YHRAW_ADDRESS 0x3D
#define MPU9250_ACCELERATION_YLRAW_ADDRESS 0x3E
#define MPU9250_ACCELERATION_ZHRAW_ADDRESS 0x3F
#define MPU9250_ACCELERATION_ZLRAW_ADDRESS 0x40
#define MPU9250_ACCEL_XH_OFFSET_ADDRESS 0x77
#define MPU9250_ACCEL_XL_OFFSET_ADDRESS 0x78
#define MPU9250_ACCEL_YH_OFFSET_ADDRESS 0x7A
#define MPU9250_ACCEL_YL_OFFSET_ADDRESS 0x7B
#define MPU9250_ACCEL_ZH_OFFSET_ADDRESS 0x7D
#define MPU9250_ACCEL_ZL_OFFSET_ADDRESS 0x7E

#define MPU9250_WHOIAM_VALUE 0x70

class mpu9250
{
  public:
    mpu9250(uint32_t i2c_clock_frequency)
    {
      Wire.setClock(i2c_clock_frequency);
      Wire.begin();
    };

    void powerStart()
    {
      write(MPU9250_POWER_MGMNT_1_ADDRESS, 0x00);
      delay(100);
    };

    bool checkConnection()
    {
      uint8_t data = getID();
      Serial.print("CHECK CONNECTION RETURN DATA:");
      Serial.println(data, HEX);
      return data == MPU9250_WHOIAM_VALUE;
    };

    uint8_t getID()
    {
      return read(MPU9250_WHOIAM_REGISTER_ADDRESS);
    };

    uint8_t isReady()
    {
      return read(MPU9250_INTERRUPT_STATUS_ADDRESS) & 0x01;
    };

    void write(uint8_t reg_addr, uint8_t data)
    {
      Wire.beginTransmission(MPU9250_DEVICE_ADDRESS);
      Wire.write(reg_addr);
      Wire.write(data);
      Wire.endTransmission(true);
    };

    uint8_t read(uint8_t reg_addr)
    {
      Wire.beginTransmission(MPU9250_DEVICE_ADDRESS);
      Wire.write(reg_addr);
      Wire.endTransmission(true);
      Wire.requestFrom(MPU9250_DEVICE_ADDRESS, 1);
      if (Wire.available()) return Wire.read();
      return 0x00;
    };

    void getRawAcceleration()
    {
      RawAcceleration[0][0] = read(MPU9250_ACCELERATION_XLRAW_ADDRESS);
      RawAcceleration[0][1] = read(MPU9250_ACCELERATION_XHRAW_ADDRESS);
      RawAcceleration[1][0] = read(MPU9250_ACCELERATION_YLRAW_ADDRESS);
      RawAcceleration[1][1] = read(MPU9250_ACCELERATION_YHRAW_ADDRESS);
      RawAcceleration[2][0] = read(MPU9250_ACCELERATION_ZLRAW_ADDRESS);
      RawAcceleration[2][1] = read(MPU9250_ACCELERATION_ZHRAW_ADDRESS);
    };
    void ConvertRawToFloat()
    {
      Raw16Acceleration[0] =  (uint16_t)RawAcceleration[0][1];
      Raw16Acceleration[0] =  Raw16Acceleration[0] << 8;
      Raw16Acceleration[0] =  Raw16Acceleration[0] + (uint16_t)RawAcceleration[0][0];
      Raw16Acceleration[1] =  (uint16_t)RawAcceleration[1][1];
      Raw16Acceleration[1] =  Raw16Acceleration[1] << 8;
      Raw16Acceleration[1] =  Raw16Acceleration[1] + (uint16_t)RawAcceleration[1][0];
      Raw16Acceleration[2] =  (uint16_t)RawAcceleration[2][1];
      Raw16Acceleration[2] =  Raw16Acceleration[2] << 8;
      Raw16Acceleration[2] =  Raw16Acceleration[2] + (uint16_t)RawAcceleration[2][0];
      SRaw16Acceleration[0] = (int16_t)Raw16Acceleration[0];
      SRaw16Acceleration[1] = (int16_t)Raw16Acceleration[1];
      SRaw16Acceleration[2] = (int16_t)Raw16Acceleration[2];
      FloatAcceleration[0] =  (float)SRaw16Acceleration[0];
      FloatAcceleration[1] =  (float)SRaw16Acceleration[1];
      FloatAcceleration[2] =  (float)SRaw16Acceleration[2];
      FloatAcceleration[0] =  FloatAcceleration[0]*2.0/(32768.0); 
      FloatAcceleration[1] =  FloatAcceleration[1]*2.0/(32768.0);
      FloatAcceleration[2] =  FloatAcceleration[2]*2.0/(32768.0);
    };
    void readAcceleration()
    {
      getRawAcceleration();
      ConvertRawToFloat();
    };
  void updateAccelerationXoffset(int data)
  {
    uint8_t msb = 0;
    uint8_t lsb = 0;
    
    // Handle signed values (2's complement)
    lsb = (uint8_t)(data & 0xFF); // LSB (lower byte)
    msb = (uint8_t)((data >> 8) & 0xFF); // MSB (upper byte)

    write(MPU9250_ACCEL_XL_OFFSET_ADDRESS, lsb);
    write(MPU9250_ACCEL_XH_OFFSET_ADDRESS, msb);
  };

  void updateAccelerationYoffset(int data)
  {
    uint8_t msb = 0;
    uint8_t lsb = 0;
    
    // Handle signed values (2's complement)
    lsb = (uint8_t)(data & 0xFF); // LSB (lower byte)
    msb = (uint8_t)((data >> 8) & 0xFF); // MSB (upper byte)

    write(MPU9250_ACCEL_YL_OFFSET_ADDRESS, lsb);
    write(MPU9250_ACCEL_YH_OFFSET_ADDRESS, msb);
  };

  void updateAccelerationZoffset(int data)
  {
    uint8_t msb = 0;
    uint8_t lsb = 0;
    
    // Handle signed values (2's complement)
    lsb = (uint8_t)(data & 0xFF); // LSB (lower byte)
    msb = (uint8_t)((data >> 8) & 0xFF); // MSB (upper byte)

    write(MPU9250_ACCEL_ZL_OFFSET_ADDRESS, lsb);
    write(MPU9250_ACCEL_ZH_OFFSET_ADDRESS, msb);
  };
  void calibrate()
  {
    int X=0,Y=0,Z=0;
    int32_t Xb=0,Yb=0,Zb=0;
    Xb = this->read(MPU9250_ACCEL_XH_OFFSET_ADDRESS);
    Xb = Xb * 256;
    Xb = Xb + this->read(MPU9250_ACCEL_XL_OFFSET_ADDRESS);
    Yb = this->read(MPU9250_ACCEL_YH_OFFSET_ADDRESS);
    Yb = Yb * 256;
    Yb = Yb + this->read(MPU9250_ACCEL_YL_OFFSET_ADDRESS);
    Zb = this->read(MPU9250_ACCEL_ZH_OFFSET_ADDRESS);
    Zb = Zb * 256;
    Zb = Zb + this->read(MPU9250_ACCEL_ZL_OFFSET_ADDRESS);
    Serial.println("Previous OFFSET:");
    Serial.println(Xb);
    Serial.println(Yb);
    Serial.println(Zb);
    Serial.println("--***---");
    for (int i = 0; i < 8; i++)
    {
      if (this->isReady())
      {
        this->readAcceleration();
      };
      X = X + (int)SRaw16Acceleration[0];
      Y = Y + (int)SRaw16Acceleration[1];
      Z = Z + (int)SRaw16Acceleration[2] - 16384;
    };
    X = (X) / 256;
    Y = (Y) / 256;
    Z = (Z) / 256;
    Serial.println("Calculated OFFSET:");
    Serial.println(X);
    Serial.println(Y);
    Serial.println(Z);
    Serial.println("--***---") ;
    if (this->isReady())
      {
        this->readAcceleration();
      };
    Serial.println("DATA BEFORE OFFSET CORRECTION:");
    Serial.println(FloatAcceleration[0]);
    Serial.println(FloatAcceleration[1]);
    Serial.println(FloatAcceleration[2]);
    Serial.println("--***---") ;
    if (X < 0)
    {
      Xb = abs(X) + Xb;
    }
    else
    {
      Xb = -abs(X) + Xb;
    };
    if (Y < 0)
    {
      Yb = abs(Y) + Yb;
    }
    else
    {
      Yb = -abs(Y) + Yb;
    };
    if (Z < 0)
    {
      Zb = abs(Z) + Zb;
    }
    else
    {
      Zb = -abs(Z) + Zb;
    };
    Serial.println("INDERMEDIATE OFFSET:");
    Serial.println(Xb);
    Serial.println(Yb);
    Serial.println(Zb);
    Serial.println("--***---");
    updateAccelerationXoffset(Xb);
    updateAccelerationYoffset(Yb);
    updateAccelerationZoffset(Zb - 16384/8);
    if (this->isReady())
      {
        this->readAcceleration();
      };
    Serial.println("DATA AFTER OFFSET CORRECTION:");
    Serial.println(FloatAcceleration[0]);
    Serial.println(FloatAcceleration[1]);
    Serial.println(FloatAcceleration[2]);
    Xb = this->read(MPU9250_ACCEL_XH_OFFSET_ADDRESS);
    Xb = Xb * 256;
    Xb = Xb + this->read(MPU9250_ACCEL_XL_OFFSET_ADDRESS);
    Yb = this->read(MPU9250_ACCEL_YH_OFFSET_ADDRESS);
    Yb = Yb * 256;
    Yb = Yb + this->read(MPU9250_ACCEL_YL_OFFSET_ADDRESS);
    Zb = this->read(MPU9250_ACCEL_ZH_OFFSET_ADDRESS);
    Zb = Zb * 256;
    Zb = Zb + this->read(MPU9250_ACCEL_ZL_OFFSET_ADDRESS);
    Serial.println("LAST OFFSET:");
    Serial.println(Xb);
    Serial.println(Yb);
    Serial.println(Zb);
    Serial.println("--***---");
  };


};





mpu9250 IMU(400000);

void setup() 
{
  Serial.begin(115200);  // Start serial communication at 115200 baud
  IMU.powerStart();  // Power start the MPU9250

  if (IMU.checkConnection()) {
    Serial.println("CONNECTION SET UP SUCCESSFULLY");
  }
  IMU.calibrate();
  FloatVelocity[0] = 0.0;
  FloatVelocity[1] = 0.0;
  FloatVelocity[2] = 0.0;
  FloatLocation[0] = 0.0;
  FloatLocation[1] = 0.0;
  FloatLocation[2] = 0.0;
}

void loop() 
{
  if (IMU.isReady()) 
  {  // Check if the IMU is ready
    print++;
    BackFloatAcceleration[0] = FloatAcceleration[0];
    BackFloatAcceleration[1] = FloatAcceleration[1];
    BackFloatAcceleration[2] = FloatAcceleration[2];
    IMU.readAcceleration();  // Read acceleration data
    if ((FloatAcceleration[0] > 0.15) || (FloatAcceleration[0] < -0.15) )
    {
      FloatVelocity[0] += (FloatAcceleration[0] - BackFloatAcceleration[0]) * 0.250;
      FloatLocation[0] += FloatVelocity[0] * 0.250;
    };
    if ((FloatAcceleration[1] > 0.15) || (FloatAcceleration[1] < -0.15) )
    {
      FloatVelocity[1] += (FloatAcceleration[1] - BackFloatAcceleration[1]) * 0.250;
      FloatLocation[1] += FloatVelocity[1] * 0.250;
    };
    if ((FloatAcceleration[2] > 0.15) || (FloatAcceleration[2] < -0.15) )
    {
      FloatVelocity[2] += (FloatAcceleration[2] - BackFloatAcceleration[2]) * 0.250;
      FloatLocation[2] += FloatVelocity[2] * 0.250;
    };
    if (print == 100)
    {
      Serial.println("X:");
      Serial.println(FloatLocation[0]);
      Serial.println("Y:");
      Serial.println(FloatLocation[1]);
      Serial.println("Z:");
      Serial.println(FloatLocation[2]);
      Serial.println("Xacce:");
      Serial.println(FloatAcceleration[0]);
      Serial.println("Yacce:");
      Serial.println(FloatAcceleration[1]);
      Serial.println("Zacce:");
      Serial.println(FloatAcceleration[2]);
      print = 0;
    };
  }
  delay(10);
}
