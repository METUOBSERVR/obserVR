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
static uint32_t   rawAccel[3];
static double     floatAccel[3];
static double     floatAccelVel[3];
static double     floatAccelLoc[3];
static uint32_t   rawGyro[3];
static double     floatGyro[3];
static double     floatGyroLoc[3];

// MPU9250 CLASS 
// ---
class MPU9250
{
  private:
    uint32_t  offsetAccel[3];
    uint32_t  offsetGyro[3];
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
      floatAccel[0] = floatAccel[0]; 
      tmp = (int16_t)rawAccel[1];
      floatAccel[1] = (float)tmp;
      floatAccel[1] = (floatAccel[1]/32768.0)*2.0;
      floatAccel[1] = floatAccel[1];
      tmp = (int16_t)rawAccel[2];
      floatAccel[2] = (float)tmp;
      floatAccel[2] = (floatAccel[2]/32768.0)*2.0;
      floatAccel[2] = floatAccel[2];
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
      floatGyro[0] = floatGyro[0];
      tmp = (int16_t)rawGyro[1];
      floatGyro[1] = (float)tmp;
      floatGyro[1] = (floatGyro[1]/32768.0)*250.0;
      floatGyro[1] = floatGyro[1]; 
      tmp = (int16_t)rawGyro[2];
      floatGyro[2] = (float)tmp;
      floatGyro[2] = (floatGyro[2]/32768.0)*250.0;
      floatGyro[2] = floatGyro[2];
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
      this->write(MPU9250_POWER_MANAGEMENT_2, 0x08);
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
class queue_32b 
{
  private:
    int numElement = 0;
    uint32_t average = 0.0;
  public:
    uint32_t data[32]; 
    queue()
    {
      for (int i = 0; i < 32; i++)
      {
        data[i] = 0.0;
      };
    };
    bool isFull()
    {
      return (numElement == 32);
    };
    bool isEmpty()
    {
      return (numElement == 0);
    };
    void insert(uint32_t data32)
    {
      uint32_t tmp = (uint32_t) data32;
      for (int i = 0; i < 31; i++)
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
      average = 0;
      for (int i = 0; i < 32; i++)
      {
        average += data[i];
      };
      average = average /32;
    };
    uint32_t returnAverage()
    {
      if (this->isFull())
      {
        return (uint32_t)this->average;
      }
      else
      { 
        return 0;
      };
    };
};
// ---


MPU9250 IMU(I2C_CLOCK_FREQUENCY);
void setup() {
  IMU.powerStart();
  Serial.begin(115200);
  #ifdef DEBUG
    Serial.println("System starting...");
    delay(2000);
  #endif
  IMU.disableYZ();
}

void loop() {
  if(IMU.isReady())
  {
    IMU.updateRawAccel();
    IMU.updateRawGyro();
    Serial.print("x");
    Serial.println(rawAccel[0], HEX);
    Serial.print("y");
    Serial.println(rawAccel[1], HEX);
    Serial.print("z");
    Serial.println(rawAccel[2], HEX);
    Serial.print("a");
    Serial.println(rawGyro[0], HEX);
    Serial.print("b");
    Serial.println(rawGyro[1], HEX);
    Serial.print("c");
    Serial.println(rawGyro[2], HEX);
  };
}
