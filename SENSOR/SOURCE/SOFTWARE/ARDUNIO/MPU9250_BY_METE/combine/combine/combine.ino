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
#define FILTER_ORDER 46  // Filter order (number of taps - 1)
float coefficients[] = {
    0.000000000000000000,
    -0.001884249596973500,
    -0.010367166384385500,
    -0.014964411705062400,
    0.009289392902356300,
    0.072286947863283600,
    0.132851905489640000,
    0.100494683086973500,
    -0.094305879612558900,
    -0.391858238545700700,
    -0.557447846675859300,
    -0.299370171182356600,
    0.461803526025833600,
    1.357105237620096500,
    1.628090370920247400,
    0.597476400174303100,
    -1.660400808788195200,
    -3.938156534710095200,
    -4.243981123452575400,
    -0.881565934833205900,
    6.310669660962553400,
    15.402413455890662200,
    23.023651639623526900,
    25.996338289854981100,
    23.023651639623529700,
    15.402413455890662200,
    6.310669660962552100,
    -0.881565934833206300,
    -4.243981123452576800,
    -3.938156534710095200,
    -1.660400808788195200,
    0.597476400174303600,
    1.628090370920247700,
    1.357105237620096100,
    0.461803526025833300,
    -0.299370171182356700,
    -0.557447846675859700,
    -0.391858238545700900,
    -0.094305879612559000,
    0.100494683086973600,
    0.132851905489640000,
    0.072286947863283700,
    0.009289392902356300,
    -0.014964411705062400,
    -0.010367166384385500,
    -0.001884249596973500,
    0.000000000000000000
};


#define SAMPLE_RATE 1000  // Sampling rate in Hz
#define MAX_SAMPLES 100  // Number of samples for processing (adjust as needed)

float bufferY[FILTER_ORDER + 1];
float bufferX[FILTER_ORDER + 1];
int bufferIndexX = 0;
int bufferIndexY = 0;



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
#define MPU9250_ACCELCNFG_2 0x1D
// ---


// GENERAL STATIC VARIABLE 
// ---
static uint16_t   rawAccel[3];
static double    floatAccel[3];
static double    prevfloatAccel[3];
static double    floatAccelVel[3];
static double    floatAccelLoc[3];
static uint16_t  rawGyro[3];
static double    floatGyro[3];
static double    floatGyroLoc[3];
static int print = 0;
static int step = 0;

static float twoKp;			// 2 * proportional gain (Kp)
static float twoKi;			// 2 * integral gain (Ki)
static float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame


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
      floatAccel[0] = (double)tmp;
      floatAccel[0] = (floatAccel[0]/32768.0)*2.0; 
      floatAccel[0] = floatAccel[0] - (double)startAccel[0]; //+ 1.0 * sin(2*PI*(floatGyroLoc[1]/360.0)); 
      tmp = (int16_t)rawAccel[1];//
      floatAccel[1] = (double)tmp;//
      floatAccel[1] = (floatAccel[1]/32768.0)*2.0;//
      floatAccel[1] = floatAccel[1] - (double)startAccel[1]; //- 1.0 * sin(2*PI*(floatGyroLoc[0]/360.0));
      tmp = (int16_t)rawAccel[2];//
      floatAccel[2] = (double)tmp;//
      floatAccel[2] = (floatAccel[2]/32768.0)*2.0;//
      floatAccel[2] = floatAccel[2] - (double)startAccel[2]; //-  1.0 * cos(2*PI*(floatGyroLoc[0]/360.0)) - 1.0 * cos(2*PI*(floatGyroLoc[1]/360.0)) + 2.0; 
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
      floatGyro[0] = (double)tmp;
      floatGyro[0] = (floatGyro[0]/32768.0)*250.0;
      floatGyro[0] = floatGyro[0] - (double)startGyro[0];
      tmp = (int16_t)rawGyro[1];
      floatGyro[1] = (double)tmp;
      floatGyro[1] = (floatGyro[1]/32768.0)*250.0;
      floatGyro[1] = floatGyro[1] - (double)startGyro[1];
      tmp = (int16_t)rawGyro[2];
      floatGyro[2] = (double)tmp;
      floatGyro[2] = (floatGyro[2]/32768.0)*250.0;
      floatGyro[2] = floatGyro[2] - (double)startGyro[2];
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
      delay(30);
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
      X = (X /256);
      Y = (Y /256);
      Z = (Z /256);
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
      delay(400);
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
      delay(200);
      this->getOffsetGyro();
      delay(200);
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
      for (int i = 0; i < 128; i++)
      {
        while(!(isReady())){};
        updateFloatAccel();
        startAccel[0] = startAccel[0] + floatAccel[0];
        startAccel[1] = startAccel[1] + floatAccel[1];
        startAccel[2] = startAccel[2] + floatAccel[2];
      };
      startAccel[0] = -startAccel[0] / 128.0;
      startAccel[1] = -startAccel[1] / 128.0;
      startAccel[2] = -startAccel[2] / 128.0;
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
      for (int i = 0; i < 128; i++)
      {
        while(!(isReady())){};
        updateFloatGyro();
        startGyro[0] = startGyro[0] + floatGyro[0];
        startGyro[1] = startGyro[1] + floatGyro[1];
        startGyro[2] = startGyro[2] + floatGyro[2];
        delay(50);
      };
      startGyro[0] = -startGyro[0] / 128.0;
      startGyro[1] = -startGyro[1] / 128.0;
      startGyro[2] = -startGyro[2] / 128.0;
      #ifdef DEBUG
        Serial.println("START OFFSET CORRECTION:");
        Serial.println(startGyro[0]);
        Serial.println(startGyro[1]);
        Serial.println(startGyro[2]);
      #endif
    };

    void change_config_accel()
    {
      this->write(MPU9250_ACCELCNFG_2, 0x0B);
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
    double data[32]; 
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
    void insert(double floatData)
    {
      double tmp = (double) floatData;
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
      average = 0.0;
      for (int i = 0; i < 32; i++)
      {
        average += data[i];
      };
      average = average / 32.0;
    };
    double returnAverage()
    {
      if (this->isFull())
      {
        return (double)this->average;
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
float applyFIRFilterX(float input);
float applyFIRFilterY(float input);
// ---
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);


MPU9250 IMU(I2C_CLOCK_FREQUENCY);
queue queueAccelX;
queue queueAccelY;
queue queueAccelZ;
queue queueGyroX;
queue queueGyroY;
queue queueGyroZ;
queue queueVelX;
queue queueVelY;
queue queueVelZ;
queue queueLocX;
queue queueLocY;
queue queueLocZ;
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
  //IMU.change_config_accel();
  IMU.offsetCorrectionAccel();
  delay(200);
  IMU.offsetCorrectionGyro();
  delay(200);
  IMU.startCorrectionAccel();
  delay(200);
  IMU.startCorrectionGyro();
  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (IMU.isReady())
  {
    IMU.updateFloatGyro();
    IMU.updateFloatAccel();
    queueAccelX.insert(floatAccel[0] - queueAccelX.returnAverage());
    queueAccelY.insert(floatAccel[1] - queueAccelY.returnAverage());
    queueAccelZ.insert(floatAccel[2] - queueAccelZ.returnAverage());
    queueGyroX.insert(floatGyro[0]);
    queueGyroY.insert(floatGyro[1]);
    queueGyroZ.insert(floatGyro[2]);
    print++;
    if (queueAccelX.isFull())
    {

    }
    if (queueAccelX.isFull())
    {
      queueAccelX.getAverage();
      floatAccelVel[0] -=  queueAccelX.returnAverage() * 0.250;
      queueVelX.insert(floatAccelVel[0]);
      queueVelX.getAverage();
      floatAccelLoc[0] += (queueVelX.returnAverage()) * 0.250;
      floatAccelVel[0] /= 10; 
      //Serial.println(queueVelX.returnAverage() - data);
      //Serial.print("x:");
      //Serial.println(floatAccelLoc[0]);
      
    }
    if (queueAccelY.isFull())
    {
      queueAccelY.getAverage();
      floatAccelVel[1] -=  queueAccelY.returnAverage() * 0.250;
      queueVelY.insert(floatAccelVel[1]);
      queueVelY.getAverage();
      floatAccelLoc[1] += (queueVelY.returnAverage()) * 0.250; 
      floatAccelVel[1] /= 10; 
      //Serial.println(queueVelX.returnAverage() - data);
      //Serial.print("y:");
      //Serial.println(floatAccelLoc[1]);
      
    }
    if (queueAccelZ.isFull())
    {
      queueAccelZ.getAverage();
      floatAccelVel[2] -= queueAccelZ.returnAverage() * 0.250;
    }
    if (queueGyroX.isFull())
    {
      queueGyroX.getAverage();
      //Serial.print("GX:");
      if (fabs(queueGyroX.returnAverage()) > 2)
      {
        floatGyroLoc[0] += queueGyroX.returnAverage() * 0.0324;
        //Serial.print("GyroX:");
        //Serial.println(floatGyroLoc[0]);
      }
      else
      {
        floatGyroLoc[0] += 0.0;
      }
    }
    if (queueGyroY.isFull())
    {
      queueGyroY.getAverage();
      //Serial.print("GY:");
      if (fabs(queueGyroY.returnAverage()) > 2)
      {
        floatGyroLoc[1] += queueGyroY.returnAverage() * 0.0324;
        //Serial.print("GyroY:");
        //Serial.println(floatGyroLoc[1]);
      }
      else
      {
        floatGyroLoc[1] += 0.0;
      }
    }
    if (queueGyroZ.isFull())
    {
      queueGyroZ.getAverage();
      //Serial.print("GZ:");
      if (fabs(queueGyroZ.returnAverage()) > 2)
      {
        floatGyroLoc[2] += queueGyroZ.returnAverage() * 0.0324;
        //Serial.print("GyroZ:");
        //Serial.println(floatGyroLoc[2]);
      }
      else
      {
        floatGyroLoc[2] += 0.0;
      }
    }
    #ifdef DEBUG
    Serial.println("DATA:");
    Serial.print("X:");
    Serial.println(floatAccel[0]);
    Serial.print("Y:");
    Serial.println(floatAccel[1]);
    Serial.print("Z:");
    Serial.println(floatAccel[2]);
    Serial.print("GX:");
    Serial.println(floatGyro[0]);
    Serial.print("GY:");
    Serial.println(floatGyro[1]);
    Serial.print("GZ:");
    Serial.println(floatGyro[2]);
    #endif
    print++;
    if (print == 100)
    {
      print = 0;
      Serial.println("LOCATION:");
      Serial.println((floatAccelLoc[0]));
      Serial.println((floatAccelLoc[1]));
      Serial.println("ANGLE:");
      Serial.println(floatGyroLoc[0]);
      Serial.println(floatGyroLoc[1]);
      Serial.println(floatGyroLoc[2]);

    }
  };
  delay(25);

}

float applyFIRFilterX(float input) {
  // Shift buffer
  bufferX[bufferIndexX] = input;
  float output = 0.0;
  
  // Calculate the filtered output
  for (int i = 0; i <= FILTER_ORDER; i++) {
    int index = (bufferIndexX - i + (FILTER_ORDER + 1)) % (FILTER_ORDER + 1);
    output += coefficients[i] * bufferX[index];
  }

  // Update buffer index
  bufferIndexX = (bufferIndexX + 1) % (FILTER_ORDER + 1);

    return output;
}
float applyFIRFilterY(float input) {
  // Shift buffer
  bufferY[bufferIndexY] = input;
  float output = 0.0;
  
  // Calculate the filtered output
  for (int i = 0; i <= FILTER_ORDER; i++) {
    int index = (bufferIndexY - i + (FILTER_ORDER + 1)) % (FILTER_ORDER + 1);
    output += coefficients[i] * bufferY[index];
  }

  // Update buffer index
  bufferIndexY = (bufferIndexY + 1) % (FILTER_ORDER + 1);

  return output;
}


// ---