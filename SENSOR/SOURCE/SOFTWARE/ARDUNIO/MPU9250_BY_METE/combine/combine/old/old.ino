// NECESSARRY INCLUDE 
// ---
#include <stdint.h>
#include <math.h>
#include <Wire.h>
// ---

// GENERAL CONSTANT FOR SYSTEM 
// ---
#define MPU9250_DEVICE_ADDR 0x68
#define MPU9250_DEVICE_ID 0x70
#define I2C_CLOCK_FREQUENCY 400000
//#define DEBUG
// ---



// GENERAL CONSTANT FOR MPU9250
// ---
#define MPU9250_POWER_MANAGEMENT_1 0x6B
#define MPU9250_POWER_MANAGEMENT_2 0x6C
#define MPU9250_WHOIAM 0x75
#define MPU9250_INTERRUPT_STATUS 0x3A
#define MPU9250_ACCEL_XH 0x3B
#define MPU9250_ACCEL_XL 0x3C
#define MPU9250_ACCEL_YH 0x3D
#define MPU9250_ACCEL_YL 0x3E
#define MPU9250_ACCEL_ZH 0x3F
#define MPU9250_ACCEL_ZL 0x40
#define MPU9250_GYRO_XH 0x43
#define MPU9250_GYRO_XL 0x44
#define MPU9250_GYRO_YH 0x45
#define MPU9250_GYRO_YL 0x46
#define MPU9250_GYRO_ZH 0x47
#define MPU9250_GYRO_ZL 0x48
#define MPU9250_ACCEL_OFFSET_XH 0x77
#define MPU9250_ACCEL_OFFSET_XL 0x78
#define MPU9250_ACCEL_OFFSET_YH 0x7A
#define MPU9250_ACCEL_OFFSET_YL 0x7B
#define MPU9250_ACCEL_OFFSET_ZH 0x7D
#define MPU9250_ACCEL_OFFSET_ZL 0x7E
#define MPU9250_GYRO_OFFSET_XH 0x13
#define MPU9250_GYRO_OFFSET_XL 0x14
#define MPU9250_GYRO_OFFSET_YH 0x15
#define MPU9250_GYRO_OFFSET_YL 0x16
#define MPU9250_GYRO_OFFSET_ZH 0x17
#define MPU9250_GYRO_OFFSET_ZL 0x18
// ---


// GENERAL STATIC VARIABLE 
// ---
static uint16_t rawAccel[3];
static float    floatAccel[3];
static float    PreviousFloatAccel[3];
static float    floatAccelVel[3];
static float    floatAccelLoc[3];
static float    FilteredfloatAccelLoc[3];
static float    SecondFilteredfloatAccelLoc[3];
static float    inputSamples[93];
static uint16_t rawGyro[3];
static float    floatGyro[3];
static float    floatGyroLoc[3];
static int print    = 0;
static int calib    = 0;
float filterCoefficients[93] = {
    0.000000000000000002,
    -0.000000000000000002,
    0.000000000598135125,
    0.000000000000000007,
    -0.000000005460708540,
    -0.000000000000000001,
    0.000000009932118071,
    0.000000000000000001,
    0.000000020448150238,
    0.000000000000000002,
    -0.000000054082152198,
    0.000000000000000004,
    -0.000000054247141676,
    -0.000000000000000007,
    0.000000178113489813,
    0.000000000000000001,
    0.000000131275955951,
    0.000000000000000001,
    -0.000000472579839186,
    -0.000000000000000002,
    -0.000000475816896126,
    -0.000000000000000002,
    0.000000790014721470,
    0.000000000000000000,
    0.000050568126508800,
    -0.000000007067912341,
    0.000000825452202147,
    0.000000033531580276,
    -0.001712761089239415,
    0.000000092773479247,
    -0.000000307613156992,
    -0.000000206418860774,
    0.008662459603143657,
    -0.000000405525953679,
    0.000000106564804366,
    0.000000733610739226,
    -0.028541715285338283,
    0.000001256652953179,
    -0.000000027267777360,
    -0.000002095952709380,
    0.080191208343845716,
    -0.000003548205540672,
    0.000000002940974838,
    0.000006696378811801,
    -0.308687348285202612,
    0.000021362808049876,
    0.500026015457524009,
    0.000021362808049855,
    -0.308687348285202390,
    0.000006696378811808,
    0.000000002940974803,
    -0.000003548205540677,
    0.080191208343845993,
    -0.000002095952709382,
    -0.000000027267777272,
    0.000001256652953178,
    -0.028541715285338297,
    0.000000733610739223,
    0.000000106564804328,
    -0.000000405525953678,
    0.008662459603143737,
    -0.000000206418860777,
    -0.000000307613156996,
    0.000000092773479247,
    -0.001712761089239426,
    0.000000033531580274,
    0.000000825452202113,
    -0.000000007067912339,
    0.000050568126508813,
    -0.000000000000000001,
    0.000000790014721458,
    0.000000000000000000,
    -0.000000475816896106,
    -0.000000000000000006,
    -0.000000472579839233,
    0.000000000000000002,
    0.000000131275955953,
    -0.000000000000000001,
    0.000000178113489821,
    -0.000000000000000001,
    -0.000000054247141678,
    0.000000000000000005,
    -0.000000054082152200,
    -0.000000000000000001,
    0.000000020448150242,
    -0.000000000000000001,
    0.000000009932118080,
    0.000000000000000002,
    -0.000000005460708535,
    -0.000000000000000002,
    0.000000000598135104,
    0.000000000000000004,
    0.000000000000000012,
};
// ---

// MPU9250 CLASS 
// ---
class MPU9250
{
  private:
    uint16_t offsetAccel[3];
    uint16_t offsetGyro[3];
    double    startAccel[3] = {0.0, 0.0, 0.0};
    double    startGyro[3]  = {0.0, 0.0, 0.0};


    void write(uint8_t reg_addr, uint8_t data)
    {
      Wire.beginTransmission(MPU9250_DEVICE_ADDR);
      Wire.write(reg_addr);
      Wire.write(data);
      Wire.endTransmission(true);
    };

    uint8_t read(uint8_t reg_addr)
    {
      Wire.beginTransmission(MPU9250_DEVICE_ADDR);
      Wire.write(reg_addr);
      Wire.endTransmission(true);
      Wire.requestFrom(MPU9250_DEVICE_ADDR ,1);
      if (Wire.available()) return Wire.read();
      return 0x00;
    };
    
    void getRawAccel()
    {
      rawAccel[0]   = read(MPU9250_ACCEL_XH);
      rawAccel[0]   = rawAccel[0] << 8;
      rawAccel[0]   = rawAccel[0] + read(MPU9250_ACCEL_XL);
      rawAccel[1]   = read(MPU9250_ACCEL_YH);
      rawAccel[1]   = rawAccel[1] << 8;
      rawAccel[1]   = rawAccel[1] + read(MPU9250_ACCEL_YL);
      rawAccel[2]   = read(MPU9250_ACCEL_ZH);
      rawAccel[2]   = rawAccel[2] << 8;
      rawAccel[2]   = rawAccel[2] + read(MPU9250_ACCEL_ZL);
    };

    void rawFloatAccel()
    {
      int16_t tmp = 0;
      tmp = (int16_t)rawAccel[0];
      floatAccel[0] = (float)tmp;
      floatAccel[0] = (floatAccel[0]/32768.0)*2.0; 
      floatAccel[0] = floatAccel[0]; //- (float)startAccel[0]; 
      tmp = (int16_t)rawAccel[1];
      floatAccel[1] = (float)tmp;
      floatAccel[1] = (floatAccel[1]/32768.0)*2.0;
      floatAccel[1] = floatAccel[1]; //- (float)startAccel[1]; 
      tmp = (int16_t)rawAccel[2];
      floatAccel[2] = (float)tmp;
      floatAccel[2] = (floatAccel[2]/32768.0)*2.0;
      floatAccel[2] = floatAccel[2]; //- (float)startAccel[2]; 
    };

    void getRawGyro()
    {
      rawGyro[0]    = read(MPU9250_GYRO_XH);
      rawGyro[0]    = rawGyro[0] << 8;
      rawGyro[0]    = rawGyro[0] + read(MPU9250_GYRO_XL);
      rawGyro[1]    = read(MPU9250_GYRO_YH);
      rawGyro[1]    = rawGyro[1] << 8;
      rawGyro[1]    = rawGyro[1] + read(MPU9250_GYRO_YL);
      rawGyro[2]    = read(MPU9250_GYRO_ZH);
      rawGyro[2]    = rawGyro[2] << 8;
      rawGyro[2]    = rawGyro[2] + read(MPU9250_GYRO_ZL);
    };

    void rawFloatGyro()
    {
      int16_t tmp = 0;
      tmp = (int16_t)rawGyro[0];
      floatGyro[0] = (float)tmp;
      floatGyro[0] = (floatGyro[0]/32768.0)*250.0;
      floatGyro[0] = floatGyro[0] ;//- (float)startGyro[0];
      tmp = (int16_t)rawGyro[1];
      floatGyro[1] = (float)tmp;
      floatGyro[1] = (floatGyro[1]/32768.0)*250.0;
      floatGyro[1] = floatGyro[1] ; //- (float)startGyro[1];
      tmp = (int16_t)rawGyro[2];
      floatGyro[2] = (float)tmp;
      floatGyro[2] = (floatGyro[2]/32768.0)*250.0;
      floatGyro[2] = floatGyro[2] ; //- (float)startGyro[2];
    };

    void getOffsetAccel()
    {
      offsetAccel[0] = read(MPU9250_ACCEL_OFFSET_XH);
      offsetAccel[0] = offsetAccel[0] << 8;
      offsetAccel[0] = offsetAccel[0] + read(MPU9250_ACCEL_OFFSET_XL);
      offsetAccel[1] = read(MPU9250_ACCEL_OFFSET_YH);
      offsetAccel[1] = offsetAccel[1] << 8;
      offsetAccel[1] = offsetAccel[1] + read(MPU9250_ACCEL_OFFSET_YL);
      offsetAccel[2] = read(MPU9250_ACCEL_OFFSET_ZH);
      offsetAccel[2] = offsetAccel[2] << 8;
      offsetAccel[2] = offsetAccel[2] + read(MPU9250_ACCEL_OFFSET_ZL);
    };

    void setOffsetAccel()
    {
      uint16_t tmp = 0;
      tmp = offsetAccel[0];
      write(MPU9250_ACCEL_OFFSET_XL, tmp);
      tmp = tmp >> 8;
      write(MPU9250_ACCEL_OFFSET_XH, tmp);
      tmp = offsetAccel[1];
      write(MPU9250_ACCEL_OFFSET_YL, tmp);
      tmp = tmp >> 8;
      write(MPU9250_ACCEL_OFFSET_YH, tmp);
      tmp = offsetAccel[2];
      write(MPU9250_ACCEL_OFFSET_ZL, tmp);
      tmp = tmp >> 8;
      write(MPU9250_ACCEL_OFFSET_ZH, tmp);
    };



    void getOffsetGyro()
    {
      offsetGyro[0]   = read(MPU9250_GYRO_OFFSET_XH);
      offsetGyro[0]   = offsetGyro[0] << 8;
      offsetGyro[0]   = offsetGyro[0] + read(MPU9250_GYRO_OFFSET_XL);
      offsetGyro[1]   = read(MPU9250_GYRO_OFFSET_YH);
      offsetGyro[1]   = offsetGyro[1] << 8;
      offsetGyro[1]   = offsetGyro[1] + read(MPU9250_GYRO_OFFSET_YL);
      offsetGyro[2]   = read(MPU9250_GYRO_OFFSET_ZH);
      offsetGyro[2]   = offsetGyro[2] << 8;
      offsetGyro[2]   = offsetGyro[2] + read(MPU9250_GYRO_OFFSET_ZL);     
    };

    void setOffsetGyro()
    {
      uint16_t tmp = 0;
      tmp = offsetGyro[0];
      write(MPU9250_GYRO_OFFSET_XL, tmp);
      tmp = tmp >> 8;
      write(MPU9250_GYRO_OFFSET_XH, tmp);
      tmp = offsetGyro[1];
      write(MPU9250_GYRO_OFFSET_YL, tmp);
      tmp = tmp >> 8;
      write(MPU9250_GYRO_OFFSET_YH, tmp);
      tmp = offsetGyro[2];
      write(MPU9250_GYRO_OFFSET_ZL, tmp);
      tmp = tmp >> 8;
      write(MPU9250_GYRO_OFFSET_ZH, tmp);
    };

  public:  
    MPU9250(uint32_t i2c_clock_freq)
    {
      Wire.setClock(i2c_clock_freq);
      Wire.begin();
    };
    
    void powerStart()
    {
      write(MPU9250_POWER_MANAGEMENT_1, 0x00);
    };

    uint8_t getID()
    {
      return read(MPU9250_WHOIAM);
    };

    uint8_t isReady()
    {
      return read(MPU9250_INTERRUPT_STATUS) & 0x01;
    };

    bool checkConnection()
    {
      return (getID() == MPU9250_DEVICE_ID);
    };

    void updateRawAccel()
    {
      getRawAccel();
    };

    void updateRawGyro()
    {
      getRawGyro();
    };

    void updateFloatAccel()
    {
      getRawAccel();
      rawFloatAccel();
      #ifdef DEBUG
        Serial.println("ACCELERATION:");
        Serial.println(floatAccel[0]);
        Serial.println(floatAccel[1]);
        Serial.println(floatAccel[2]);
        delay(2000);
      #endif
    };

    void updateFloatGyro()
    {
      getRawGyro();
      rawFloatGyro();
      #ifdef DEBUG
        Serial.println("GYRO:");
        Serial.println(floatGyro[0]);
        Serial.println(floatGyro[1]);
        Serial.println(floatGyro[2]);
        delay(2000);
      #endif
    };

    void offsetCorrectionAccel()
    {
      int X = 0, Y = 0, Z = 0;
      uint16_t Xb, Yb, Zb;
      int i = 0;
      while (i < 16)
      {
        if (isReady())
        {
          i++;
          updateRawAccel();
        }
        else
        {
          continue;
        };
        X += (int16_t)rawAccel[0];
        Y += (int16_t)rawAccel[1];
        Z += (int16_t)rawAccel[2] - 16384;
      }; 
      #ifdef DEBUG
        this->updateFloatAccel();
        Serial.println("BEFORE DIVISION FUNCTION:");
        Serial.println(X);
        Serial.println(Y);
        Serial.println(Z);
      #endif
      X = (X /128);
      Y = (Y /128);
      Z = (Z /128);
      #ifdef DEBUG
        this->updateFloatAccel();
        Serial.println("DATA BEFORE CORRECTION:");
        Serial.println(floatAccel[0]);
        Serial.println(floatAccel[1]);
        Serial.println(floatAccel[2]);
        Serial.println("ACCELERATION OFFSET CORRECTION FUNCTION:");
        Serial.println(X);
        Serial.println(Y);
        Serial.println(Z);
      #endif
      Xb = (uint16_t)abs(X);
      Yb = (uint16_t)abs(Y);
      Zb = (uint16_t)abs(Z);
      #ifdef DEBUG
        Serial.println("ABSOLUTE ACCELERATION OFFSET CORRECTION FUNCTION:");
        Serial.println(Xb);
        Serial.println(Yb);
        Serial.println(Zb);
      #endif
      getOffsetAccel();
      #ifdef DEBUG
        Serial.println("INITIAL OFFSET VALUE:");
        Serial.println(offsetAccel[0]);
        Serial.println(offsetAccel[1]);
        Serial.println(offsetAccel[2]);
      #endif
      if (X > 0)
      {
        offsetAccel[0] = offsetAccel[0] - Xb;
      }
      else
      {
        offsetAccel[0] = offsetAccel[0] + Xb;
      };
      if (Y > 0)
      {
        offsetAccel[1] = offsetAccel[1] - Yb;
      }
      else
      {
        offsetAccel[1] = offsetAccel[1] + Yb;
      };
      if (Z > 0)
      {
        offsetAccel[2] = offsetAccel[2] - Zb;
      }
      else
      {
        offsetAccel[2] = offsetAccel[2] + Zb;
      };
      offsetAccel[2] = offsetAccel[2] - (16384/8);
      setOffsetAccel();
      getOffsetAccel();
      #ifdef DEBUG
        this->updateFloatAccel();
        Serial.println("DATA AFTER CORRECTION:");
        Serial.println(floatAccel[0]);
        Serial.println(floatAccel[1]);
        Serial.println(floatAccel[2]);
        Serial.println("AFTER OFFSET VALUE:");
        Serial.println(offsetAccel[0]);
        Serial.println(offsetAccel[1]);
        Serial.println(offsetAccel[2]);
      #endif
    };
void offsetCorrectionAccel2()
    {
      int X = 0, Y = 0, Z = 0;
      uint16_t Xb, Yb, Zb;
      int i = 0;
      while (i < 16)
      {
        if (isReady())
        {
          i++;
          updateRawAccel();
        }
        else
        {
          continue;
        };
        X += (int16_t)rawAccel[0];
        Y += (int16_t)rawAccel[1];
        Z += (int16_t)rawAccel[2];
      }; 
      #ifdef DEBUG
        this->updateFloatAccel();
        Serial.println("BEFORE DIVISION FUNCTION:");
        Serial.println(X);
        Serial.println(Y);
        Serial.println(Z);
      #endif
      X = (X /128);
      Y = (Y /128);
      Z = (Z /128);
      #ifdef DEBUG
        this->updateFloatAccel();
        Serial.println("DATA BEFORE CORRECTION:");
        Serial.println(floatAccel[0]);
        Serial.println(floatAccel[1]);
        Serial.println(floatAccel[2]);
        Serial.println("ACCELERATION OFFSET CORRECTION FUNCTION:");
        Serial.println(X);
        Serial.println(Y);
        Serial.println(Z);
      #endif
      Xb = (uint16_t)abs(X);
      Yb = (uint16_t)abs(Y);
      Zb = (uint16_t)abs(Z);
      #ifdef DEBUG
        Serial.println("ABSOLUTE ACCELERATION OFFSET CORRECTION FUNCTION:");
        Serial.println(Xb);
        Serial.println(Yb);
        Serial.println(Zb);
      #endif
      getOffsetAccel();
      #ifdef DEBUG
        Serial.println("INITIAL OFFSET VALUE:");
        Serial.println(offsetAccel[0]);
        Serial.println(offsetAccel[1]);
        Serial.println(offsetAccel[2]);
      #endif
      if (X > 0)
      {
        offsetAccel[0] = offsetAccel[0] - Xb;
      }
      else
      {
        offsetAccel[0] = offsetAccel[0] + Xb;
      };
      if (Y > 0)
      {
        offsetAccel[1] = offsetAccel[1] - Yb;
      }
      else
      {
        offsetAccel[1] = offsetAccel[1] + Yb;
      };
      if (Z > 0)
      {
        offsetAccel[2] = offsetAccel[2] - Zb;
      }
      else
      {
        offsetAccel[2] = offsetAccel[2] + Zb;
      };
      offsetAccel[2] = offsetAccel[2];
      setOffsetAccel();
      getOffsetAccel();
      #ifdef DEBUG
        this->updateFloatAccel();
        Serial.println("DATA AFTER CORRECTION:");
        Serial.println(floatAccel[0]);
        Serial.println(floatAccel[1]);
        Serial.println(floatAccel[2]);
        Serial.println("AFTER OFFSET VALUE:");
        Serial.println(offsetAccel[0]);
        Serial.println(offsetAccel[1]);
        Serial.println(offsetAccel[2]);
      #endif
    };
    void offsetCorrectionGyro()
    {
      int X = 0, Y = 0, Z = 0;
      uint16_t Xb, Yb, Zb;
      int i = 0;
      while (i < 32)
      {
        if (isReady())
        {
          i++;
          updateRawGyro();
        }
        else
        {
          continue;
        };
        X += (int16_t)rawGyro[0];
        Y += (int16_t)rawGyro[1];
        Z += (int16_t)rawGyro[2];
      };
      #ifdef DEBUG
        Serial.println("BEFORE DIVISION FUNCTION:");
        Serial.println(X);
        Serial.println(Y);
        Serial.println(Z);
      #endif
      X = (X /128);
      Y = (Y /128);
      Z = (Z /128);
      #ifdef DEBUG
        this->updateFloatGyro();
        Serial.println("DATA BEFORE CORRECTION:");
        Serial.println(floatGyro[0]);
        Serial.println(floatGyro[1]);
        Serial.println(floatGyro[2]);
        Serial.println("GYRO OFFSET CORRECTION FUNCTION:");
        Serial.println(X);
        Serial.println(Y);
        Serial.println(Z);
      #endif
      Xb = (uint16_t)abs(X);
      Yb = (uint16_t)abs(Y);
      Zb = (uint16_t)abs(Z);
      #ifdef DEBUG
        Serial.println("ABSOLUTE GYRO OFFSET CORRECTION FUNCTION:");
        Serial.println(Xb);
        Serial.println(Yb);
        Serial.println(Zb);
      #endif
      getOffsetGyro();
      #ifdef DEBUG
        Serial.println("INITIAL OFFSET VALUE:");
        Serial.println(offsetGyro[0]);
        Serial.println(offsetGyro[1]);
        Serial.println(offsetGyro[2]);
      #endif
      if (X > 0)
      {
        offsetGyro[0] = offsetGyro[0] - Xb;
      }
      else
      {
        offsetGyro[0] = offsetGyro[0] + Xb;
      };
      if (Y > 0)
      {
        offsetGyro[1] = offsetGyro[1] - Yb;
      }
      else
      {
        offsetGyro[1] = offsetGyro[1] + Yb;
      };
      if (Z > 0)
      {
        offsetGyro[2] = offsetGyro[2] - Zb;
      }
      else
      {
        offsetGyro[2] = offsetGyro[2] + Zb;
      };
      this->setOffsetGyro();
      this->getOffsetGyro();
      #ifdef DEBUG
        this->updateFloatGyro();
        Serial.println("DATA AFTER CORRECTION:");
        Serial.println(floatGyro[0]);
        Serial.println(floatGyro[1]);
        Serial.println(floatGyro[2]);
        Serial.println("AFTER OFFSET VALUE:");
        Serial.println(offsetGyro[0]);
        Serial.println(offsetGyro[1]);
        Serial.println(offsetGyro[2]);
      #endif
    };

    void disableYZ()
    {
      this->write(MPU9250_POWER_MANAGEMENT_2, 0x18);
    };

    void startCorrectionAccel()
    {
      for (int i = 0; i < 8; i++)
      {
        while(!(isReady())){};
        updateFloatAccel();
        startAccel[0] = startAccel[0] + floatAccel[0];
        startAccel[1] = startAccel[1] + floatAccel[1];
        startAccel[2] = startAccel[2] + floatAccel[2];
      };
      startAccel[0] = startAccel[0] / 8.0;
      startAccel[1] = startAccel[1] / 8.0;
      startAccel[2] = startAccel[2] / 8.0;
      #ifdef DEBUG
        Serial.println("START OFFSET CORRECTION:");
        Serial.println(startAccel[0]);
        Serial.println(startAccel[1]);
        Serial.println(startAccel[2]);
        delay(50);
      #endif
    }; 

    void startCorrectionGyro()
    {
      for (int i = 0; i < 8; i++)
      {
        while(!(isReady())){};
        updateFloatGyro();
        startGyro[0] = startGyro[0] + floatGyro[0];
        startGyro[1] = startGyro[1] + floatGyro[1];
        startGyro[2] = startGyro[2] + floatGyro[2];
        delay(50);
      };
      startGyro[0] = startGyro[0] / 8.0;
      startGyro[1] = startGyro[1] / 8.0;
      startGyro[2] = startGyro[2] / 8.0;
      #ifdef DEBUG
        Serial.println("START OFFSET CORRECTION:");
        Serial.println(startGyro[0]);
        Serial.println(startGyro[1]);
        Serial.println(startGyro[2]);
      #endif
    };
};
// ---


// QUEUE FOR SYSTEM 
// ---
class queue 
{
  private:
    int numElement = 0;
    double average = 0.0;
  public:
    double data[16]; 
    queue()
    {
      for (int i = 0; i < 16; i++)
      {
        data[i] = 0.0;
      };
    };
    bool isFull()
    {
      return (numElement == 16);
    };
    bool isEmpty()
    {
      return (numElement == 0);
    };
    void insert(float floatData)
    {
      double tmp = (double) floatData;
      for (int i = 0; i < 15; i++)
      {
        data[i + 1] = data[i];
      };
      data[0] = tmp;
      if (!this->isFull()) 
      {
        numElement++;
      };
    };
    void getAverage()
    {
      average = 0.0;
      for (int i = 0; i < 16; i++)
      {
        average += data[i];
      };
      average = average / 16.0;
    };
    float returnAverage()
    {
      if (this->isFull())
      {
        return (float)this->average;
      }
      else
      { 
        return 0.0;
      };
    };
};
// ---



// GENERAL FUNCTION DECLERATION
// ---
void updateAccelVel();
void updateAccelLoc();
void updateGyroLoc();
float kalmanFilter(float loc, float velocity, float acceleration, float dt);
float applyFIRFilter(float newSample);
// ---



MPU9250 IMU(I2C_CLOCK_FREQUENCY);
queue   queueXaccel;
queue   queueYaccel;
queue   queueZaccel;
void setup() 
{
  IMU.powerStart();
  Serial.begin(115200);
  #ifdef DEBUG
    Serial.println("System starting...");
    delay(2000);
  #endif

  
  delay(200);     // Delay for sensor

  #ifdef DEBUG    // Connection Check
    if(IMU.checkConnection())
    {
      Serial.println("Sensor connection is succesful");
    }
    else
    {
      Serial.println("Sensor connection is not succesful");
    };
    Serial.print("ID:");
    Serial.println(IMU.getID(), HEX);
  #endif

  IMU.offsetCorrectionAccel();
  delay(200);
  IMU.offsetCorrectionGyro();
  delay(200);
  IMU.startCorrectionAccel();
  delay(200);
  IMU.startCorrectionGyro();
  delay(200);
  for (int i = 0; i < 93; i++)
  {
    inputSamples[i] = 0.0;
  };
  //IMU.disableYZ();

}

void loop() {
  // put your main code here, to run repeatedly:
  if (IMU.isReady())
  {
    PreviousFloatAccel[0] = floatAccel[0];
    PreviousFloatAccel[1] = floatAccel[1];
    PreviousFloatAccel[2] = floatAccel[2];
    IMU.updateFloatGyro();
    IMU.updateFloatAccel();
    updateAccelVel();
    updateAccelLoc();
    updateGyroLoc();
    print++;
    calib++;
  };
  if (calib == 1000)
  {
    //IMU.offsetCorrectionAccel2();
  };
  if (print == 100)
  {
    print = 0;
    Serial.println("DATA:");
    Serial.println(floatAccelLoc[0]);
    Serial.println(floatAccelLoc[1]);
    Serial.println(floatAccelLoc[2]);
   //Serial.println("ACCELERATION:");
   //Serial.println("Location:");
   //Serial.print("X:");
   //Serial.println(floatAccelLoc[0]);
   //Serial.print("Y:");
   //Serial.println(floatAccelLoc[1]);
   //Serial.print("Z:");
   //Serial.println(floatAccelLoc[2]);
   //Serial.println("Filtered Location:");
   //Serial.print("X:");
   //Serial.println(FilteredfloatAccelLoc[0]);
   //Serial.print("Y:");
   //Serial.println(FilteredfloatAccelLoc[1]);
   //Serial.print("Z:");
   //Serial.println(FilteredfloatAccelLoc[2]); 
   //Serial.println("Second Filtered Location:");
   //Serial.print("X:");
   //Serial.println(SecondFilteredfloatAccelLoc[0]);
   //Serial.print("Y:");
   //Serial.println(SecondFilteredfloatAccelLoc[1]);
   //Serial.print("Z:");
   //Serial.println(SecondFilteredfloatAccelLoc[2]); 
   //Serial.println("Velocity:");
   //Serial.print("X:");
   //Serial.println(queueXaccel.returnAverage());
   //Serial.print("Y:");
   //Serial.println(queueYaccel.returnAverage());
   //Serial.print("Z:");
   //Serial.println(queueZaccel.returnAverage());
   //Serial.println("Acceleration:");
   //Serial.print("X:");
   //Serial.println(floatAccel[0]);
   //Serial.print("Y:");
   //Serial.println(floatAccel[1]);
   //Serial.print("Z:");
   //Serial.println(floatAccel[2]);
   //Serial.println("GYRO");
   //Serial.println("Location:");
   //Serial.print("X:");
   //Serial.println(floatGyroLoc[0]);
   //Serial.print("Y:");
   //Serial.println(floatGyroLoc[1]);
   //Serial.print("Z:");
   //Serial.println(floatGyroLoc[2]);
   //Serial.println("Rate:");
   //Serial.print("X:");
   //Serial.println(floatGyro[0]);
   //Serial.print("Y:");
   //Serial.println(floatGyro[1]);
   //Serial.print("Z:");
   //Serial.println(floatGyro[2]);
  }


}


// GENERAL FUNCTION IMPLEMENTATION
// ---
void updateAccelVel()
{
    if (fabs(floatAccel[0]) > 0.0)
    {
      floatAccelVel[0] += (floatAccel[0] - PreviousFloatAccel[0]) * 0.250;
    }
    else
    {
      floatAccelVel[0] = 0.0 * 0.250;
    };
    if (fabs(floatAccel[1]) > 0.0)
    {
      floatAccelVel[1] += (floatAccel[1] - PreviousFloatAccel[1]) * 0.250;
    }
    else
    {
      floatAccelVel[1] = 0.0 * 0.250;
    };
    if (fabs(floatAccel[2]) > 0.0)
    {
      floatAccelVel[2] += (floatAccel[2] - PreviousFloatAccel[2]) * 0.250;
    }
    else
    {
      floatAccelVel[2] = 0.0 * 0.250;
    };
    queueXaccel.insert(floatAccelVel[0]);
    queueYaccel.insert(floatAccelVel[1]);
    queueZaccel.insert(floatAccelVel[2]);
    queueXaccel.getAverage();
    queueYaccel.getAverage();
    queueZaccel.getAverage();
};

void updateAccelLoc()
{
  floatAccelLoc[0] += 0.250 * queueXaccel.returnAverage();
  floatAccelLoc[1] += 0.250 * queueYaccel.returnAverage();
  floatAccelLoc[2] += 0.250 * queueZaccel.returnAverage();
  FilteredfloatAccelLoc[0] = kalmanFilter(floatAccelLoc[0], queueXaccel.returnAverage(), floatAccel[0], 0.0250);
  FilteredfloatAccelLoc[1] = kalmanFilter(floatAccelLoc[1], queueYaccel.returnAverage(), floatAccel[1], 0.0250);
  FilteredfloatAccelLoc[2] = kalmanFilter(floatAccelLoc[2], queueZaccel.returnAverage(), floatAccel[2], 0.0250);
  SecondFilteredfloatAccelLoc[0] = applyFIRFilter(FilteredfloatAccelLoc[0]);
  SecondFilteredfloatAccelLoc[1] = applyFIRFilter(FilteredfloatAccelLoc[1]);
  SecondFilteredfloatAccelLoc[2] = applyFIRFilter(FilteredfloatAccelLoc[2]);
  #ifdef DEBUG
    Serial.println("LAST ACCELERATION OUTPUT:");
    Serial.println("Velocity:");
    Serial.println(queueXaccel.returnAverage());

    Serial.println(queueYaccel.returnAverage());
    Serial.println(queueZaccel.returnAverage());
    Serial.println("Location:");
    Serial.println(floatAccelLoc[0]);
    Serial.println(floatAccelLoc[1]);
    Serial.println(floatAccelLoc[2]);
    delay(100);
  #endif 
};

void updateGyroLoc()
{
  if (fabs(floatGyro[0]) > 2)
  {
    floatGyroLoc[0] += 0.0270 * floatGyro[0];
  };

  if (fabs(floatGyro[1]) > 2)
  {
    floatGyroLoc[1] += 0.0270 * floatGyro[1];
  };

  if (fabs(floatGyro[2]) > 2)
  {
    floatGyroLoc[2] += 0.0270 * floatGyro[2];
  };
  #ifdef DEBUG
    Serial.println("LAST GYRO OUTPUT:");
    Serial.println("Location:");
    Serial.println(floatGyroLoc[0]);
    Serial.println(floatGyroLoc[1]);
    Serial.println(floatGyroLoc[2]);
    delay(100);
  #endif   
};


// Kalman filter function to update location based on acceleration
float kalmanFilter(float loc, float velocity, float acceleration, float dt) {
  // Kalman filter variables
  static float P[2][2] = {{1, 0}, {0, 1}}; // Covariance matrix
  static float Q[2][2] = {{0.001, 0}, {0, 0.001}}; // Process noise covariance
  static float R = 0.5;  // Measurement noise covariance
  static float K[2];  // Kalman gain
  static float x[2] = {0, 0}; // Initial state [position, velocity]

  // Initialize the state with the current values if it's the first time
  if (x[0] == 0 && x[1] == 0) {
    x[0] = loc;  // Set initial location
    x[1] = velocity;  // Set initial velocity
  }

  // Prediction Step
  // State transition: position = position + velocity * dt, velocity = velocity + acceleration * dt
  x[0] = loc + velocity * dt;  // Update position prediction
  x[1] = velocity + acceleration * dt;  // Update velocity prediction

  // Covariance prediction
  P[0][0] = P[0][0] + dt * (P[0][1] + P[1][0]) + Q[0][0];
  P[0][1] = P[0][1] + dt * P[1][1];
  P[1][0] = P[1][0] + dt * P[1][1];
  P[1][1] = P[1][1] + Q[1][1];

  // Measurement Step (correct the position/velocity based on noisy acceleration)
  // Compute Kalman Gain
  float S = P[0][0] + R;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Update state estimate
  float z = acceleration;  // Measured acceleration (using this as the measurement)
  float y = z - x[1];  // Measurement residual (difference between predicted and actual)

  x[0] = x[0] + K[0] * y;  // Update position estimate
  x[1] = x[1] + K[1] * y;  // Update velocity estimate

  // Update covariance matrix
  P[0][0] = P[0][0] - K[0] * P[0][0];
  P[0][1] = P[0][1] - K[0] * P[0][1];
  P[1][0] = P[1][0] - K[1] * P[0][0];
  P[1][1] = P[1][1] - K[1] * P[0][1];

  // Return the updated position (loc)
  return x[0];
};

float applyFIRFilter(float newSample) {
  // Shift the samples to make space for the new sample
  for (int i = 93 - 1; i > 0; i--) {
    inputSamples[i] = inputSamples[i - 1];
  }
  inputSamples[0] = newSample;  // Store the new sample
  
  // Apply the FIR filter
  float filteredOutput = 0.0;
  for (int i = 0; i < 93; i++) {
    filteredOutput += filterCoefficients[i] * inputSamples[i];
  }
  
  // Return the filtered value
  return filteredOutput;
}

// ---