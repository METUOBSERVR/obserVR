#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa myIMU(0x68); // MPU-9250 at I2C address 0x68

// Variables to hold accelerometer and gyroscope offsets
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

const int num_samples = 5; // Number of samples for calibration

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize accelerometer, gyroscope, and magnetometer
  myIMU.beginAccel();
  myIMU.beginGyro();
  myIMU.beginMag(); // Initialize magnetometer
  
  Serial.println("Accelerometer, Gyroscope, and Magnetometer initialized.");

  // Calibrate the accelerometer and gyroscope
  calibrateSensors();
  Serial.println("Calibration complete.");
}

void loop() {
  // Update sensor data
  myIMU.accelUpdate();
  myIMU.gyroUpdate();
  myIMU.magUpdate();

  // Get calibrated accelerometer data
  float ax = myIMU.accelX() - ax_offset;
  float ay = myIMU.accelY() - ay_offset;
  float az = myIMU.accelZ() - az_offset;

  // Get calibrated gyroscope data
  float gx = myIMU.gyroX() - gx_offset;
  float gy = myIMU.gyroY() - gy_offset;
  float gz = myIMU.gyroZ() - gz_offset;

  // Print calibrated accelerometer data
  Serial.print("Accel X: "); Serial.print(ax); Serial.print(" m/s^2\t");
  Serial.print("Accel Y: "); Serial.print(ay); Serial.print(" m/s^2\t");
  Serial.print("Accel Z: "); Serial.print(az); Serial.println(" m/s^2");

  // Print calibrated gyroscope data
  Serial.print("Gyro X: "); Serial.print(gx); Serial.print(" rad/s\t");
  Serial.print("Gyro Y: "); Serial.print(gy); Serial.print(" rad/s\t");
  Serial.print("Gyro Z: "); Serial.print(gz); Serial.println(" rad/s");

  // Print magnetometer data
  Serial.print("Mag X: "); Serial.print(myIMU.magX()); Serial.print(" µT\t");
  Serial.print("Mag Y: "); Serial.print(myIMU.magY()); Serial.print(" µT\t");
  Serial.print("Mag Z: "); Serial.print(myIMU.magZ()); Serial.println(" µT");

  Serial.println();
  delay(250); // Add delay for readability
}

void calibrateSensors() {
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  // Collect samples for calibration
  for (int i = 0; i < num_samples; i++) {
    myIMU.accelUpdate();
    myIMU.gyroUpdate();

    // Sum the accelerometer data
    ax_sum += myIMU.accelX();
    ay_sum += myIMU.accelY();
    az_sum += myIMU.accelZ();

    // Sum the gyroscope data
    gx_sum += myIMU.gyroX();
    gy_sum += myIMU.gyroY();
    gz_sum += myIMU.gyroZ();

    delay(100); // Add a short delay between samples
  }

  // Calculate the average (bias) for each axis
  ax_offset = ax_sum / num_samples;
  ay_offset = ay_sum / num_samples;
  az_offset = az_sum / num_samples;

  gx_offset = gx_sum / num_samples;
  gy_offset = gy_sum / num_samples;
  gz_offset = gz_sum / num_samples;

  // For accelerometer calibration, adjust for gravity on the Z-axis
  az_offset = az_offset - 1.0; // Assuming the sensor is lying flat with gravity pointing to the Z-axis
}