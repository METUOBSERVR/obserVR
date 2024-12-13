#include <Wire.h>
#include <stdint.h>
#include <math.h>


#define MPU9250_DEVICE_ADDRESS 0x68

static uint8_t  RawGyro[3][2];
static uint16_t Raw16Gyro[3];
static int16_t  SRaw16Gyro[3];
static float    FloatGyro[3];
static float    FloatVelocity[3];

static int print = 0;

volatile  bool check = false;

#define MPU9250_WHOIAM_REGISTER_ADDRESS 0x75
#define MPU9250_INTERRUPT_CONFIG_ADDRESS 0x37
#define MPU9250_INTERRUPT_ENABLE_ADDRESS 0x38
#define MPU9250_INTERRUPT_STATUS_ADDRESS 0x3A
#define MPU9250_POWER_MGMNT_1_ADDRESS 0x6B
#define MPU9250_POWER_MGMNT_2_ADDRESS 0x6C
#define MPU9250_GYRO_XHRAW_ADDRESS 0x43
#define MPU9250_GYRO_XLRAW_ADDRESS 0x44
#define MPU9250_GYRO_YHRAW_ADDRESS 0x45
#define MPU9250_GYRO_YLRAW_ADDRESS 0x46
#define MPU9250_GYRO_ZHRAW_ADDRESS 0x47
#define MPU9250_GYRO_ZLRAW_ADDRESS 0x48
#define MPU9250_GYRO_XH_OFFSET_ADDRESS 0x13
#define MPU9250_GYRO_XL_OFFSET_ADDRESS 0x14
#define MPU9250_GYRO_YH_OFFSET_ADDRESS 0x15
#define MPU9250_GYRO_YL_OFFSET_ADDRESS 0x16
#define MPU9250_GYRO_ZH_OFFSET_ADDRESS 0x17
#define MPU9250_GYRO_ZL_OFFSET_ADDRESS 0x18

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

    void getRawGyro()
    {
      RawGyro[0][0] = read(MPU9250_GYRO_XLRAW_ADDRESS);
      RawGyro[0][1] = read(MPU9250_GYRO_XHRAW_ADDRESS);
      RawGyro[1][0] = read(MPU9250_GYRO_YLRAW_ADDRESS);
      RawGyro[1][1] = read(MPU9250_GYRO_YHRAW_ADDRESS);
      RawGyro[2][0] = read(MPU9250_GYRO_ZLRAW_ADDRESS);
      RawGyro[2][1] = read(MPU9250_GYRO_ZHRAW_ADDRESS);
    };
    void ConvertRawToFloat()
    {
      Raw16Gyro[0] =  (uint16_t)RawGyro[0][1];
      Raw16Gyro[0] =  Raw16Gyro[0] << 8;
      Raw16Gyro[0] =  Raw16Gyro[0] + (uint16_t)RawGyro[0][0];
      Raw16Gyro[1] =  (uint16_t)RawGyro[1][1];
      Raw16Gyro[1] =  Raw16Gyro[1] << 8;
      Raw16Gyro[1] =  Raw16Gyro[1] + (uint16_t)RawGyro[1][0];
      Raw16Gyro[2] =  (uint16_t)RawGyro[2][1];
      Raw16Gyro[2] =  Raw16Gyro[2] << 8;
      Raw16Gyro[2] =  Raw16Gyro[2] + (uint16_t)RawGyro[2][0];
      SRaw16Gyro[0] = (int16_t)Raw16Gyro[0];
      SRaw16Gyro[1] = (int16_t)Raw16Gyro[1];
      SRaw16Gyro[2] = (int16_t)Raw16Gyro[2];
      FloatGyro[0] =  (float)SRaw16Gyro[0];
      FloatGyro[1] =  (float)SRaw16Gyro[1];
      FloatGyro[2] =  (float)SRaw16Gyro[2];
      FloatGyro[0] =  FloatGyro[0]*250.0/(32768.0); 
      FloatGyro[1] =  FloatGyro[1]*250.0/(32768.0);
      FloatGyro[2] =  FloatGyro[2]*250.0/(32768.0);
    };
    void readGyro()
    {
      getRawGyro();
      ConvertRawToFloat();
    };
  void updateGyroXoffset(int data)
  {
    uint8_t msb = 0;
    uint8_t lsb = 0;
    
    // Handle signed values (2's complement)
    lsb = (uint8_t)(data & 0xFF); // LSB (lower byte)
    msb = (uint8_t)((data >> 8) & 0xFF); // MSB (upper byte)

    write(MPU9250_GYRO_XL_OFFSET_ADDRESS, lsb);
    write(MPU9250_GYRO_XH_OFFSET_ADDRESS, msb);
  };

  void updateGyroYoffset(int data)
  {
    uint8_t msb = 0;
    uint8_t lsb = 0;
    
    // Handle signed values (2's complement)
    lsb = (uint8_t)(data & 0xFF); // LSB (lower byte)
    msb = (uint8_t)((data >> 8) & 0xFF); // MSB (upper byte)

    write(MPU9250_GYRO_YL_OFFSET_ADDRESS, lsb);
    write(MPU9250_GYRO_YH_OFFSET_ADDRESS, msb);
  };

  void updateGyroZoffset(int data)
  {
    uint8_t msb = 0;
    uint8_t lsb = 0;
    
    // Handle signed values (2's complement)
    lsb = (uint8_t)(data & 0xFF); // LSB (lower byte)
    msb = (uint8_t)((data >> 8) & 0xFF); // MSB (upper byte)

    write(MPU9250_GYRO_ZL_OFFSET_ADDRESS, lsb);
    write(MPU9250_GYRO_ZH_OFFSET_ADDRESS, msb);
  };
  void calibrate()
  {
    int X=0,Y=0,Z=0;
    int32_t Xb=0,Yb=0,Zb=0;
    Xb = this->read(MPU9250_GYRO_XH_OFFSET_ADDRESS);
    Xb = Xb * 256;
    Xb = Xb + this->read(MPU9250_GYRO_XL_OFFSET_ADDRESS);
    Yb = this->read(MPU9250_GYRO_YH_OFFSET_ADDRESS);
    Yb = Yb * 256;
    Yb = Yb + this->read(MPU9250_GYRO_YL_OFFSET_ADDRESS);
    Zb = this->read(MPU9250_GYRO_ZH_OFFSET_ADDRESS);
    Zb = Zb * 256;
    Zb = Zb + this->read(MPU9250_GYRO_ZL_OFFSET_ADDRESS);
    Serial.println("Previous OFFSET:");
    Serial.println(Xb);
    Serial.println(Yb);
    Serial.println(Zb);
    Serial.println("--***---");
    for (int i = 0; i < 8; i++)
    {
      if (this->isReady())
      {
        this->readGyro();
      };
      X = X + (int)SRaw16Gyro[0];
      Y = Y + (int)SRaw16Gyro[1];
      Z = Z + (int)SRaw16Gyro[2];
    };
    X = (X) / 32;
    Y = (Y) / 32;
    Z = (Z) / 32;
    Serial.println("Calculated OFFSET:");
    Serial.println(X);
    Serial.println(Y);
    Serial.println(Z);
    Serial.println("--***---") ;
    if (this->isReady())
      {
        this->readGyro();
      };
    Serial.println("DATA BEFORE OFFSET CORRECTION:");
    Serial.println(FloatGyro[0]);
    Serial.println(FloatGyro[1]);
    Serial.println(FloatGyro[2]);
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
    updateGyroXoffset((Xb));
    updateGyroYoffset((Yb));
    updateGyroZoffset((Zb));
    if (this->isReady())
      {
        this->readGyro();
      };
    Serial.println("DATA AFTER OFFSET CORRECTION:");
    Serial.println(FloatGyro[0]);
    Serial.println(FloatGyro[1]);
    Serial.println(FloatGyro[2]);
    Xb = this->read(MPU9250_GYRO_XH_OFFSET_ADDRESS);
    Xb = Xb * 256;
    Xb = Xb + this->read(MPU9250_GYRO_XL_OFFSET_ADDRESS);
    Yb = this->read(MPU9250_GYRO_YH_OFFSET_ADDRESS);
    Yb = Yb * 256;
    Yb = Yb + this->read(MPU9250_GYRO_YL_OFFSET_ADDRESS);
    Zb = this->read(MPU9250_GYRO_ZH_OFFSET_ADDRESS);
    Zb = Zb * 256;
    Zb = Zb + this->read(MPU9250_GYRO_ZL_OFFSET_ADDRESS);
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
}

void loop() 
{
  if (IMU.isReady()) 
  {  // Check if the IMU is ready
    IMU.readGyro();
    if ((FloatGyro[0] > 1.5) || (FloatGyro[0] < -1.5))
    {
      FloatVelocity[0] += FloatGyro[0] * 0.0125;
    };
    if ((FloatGyro[1] > 1.5) || (FloatGyro[1] < -1.5))
    {
      FloatVelocity[1] += FloatGyro[1] * 0.0125;
    };
    if ((FloatGyro[2] > 1.5) || (FloatGyro[2] < -1.5))
    {
      FloatVelocity[2] += FloatGyro[2] * 0.0125;
    };
    print++;
  }
  if (print% 100 == 0)
  { 
    Serial.println("Data:");
    Serial.println(FloatVelocity[0]);
    Serial.println(FloatVelocity[1]);
    Serial.println(FloatVelocity[2]);
  };
  if (print == 5000)
  {
    print = 0;
    IMU.calibrate();
  };
  delay(10);
}
