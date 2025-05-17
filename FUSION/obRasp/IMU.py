import time
import busio
import numpy as np
import adafruit_bno055
import threading

class BNO055Observer:
    CONFIG_MODE = 0x00
    NDOF_MODE = 0x0C

    def __init__(self, sample_amount=1000):
        self.i2c = busio.I2C(scl=3, sda=2)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.sample_amount = sample_amount

        # Internal state
        self.linearAccelBiasX = 0
        self.linearAccelBiasY = 0
        self.linearAccelBiasZ = 0
        self.linearEulerBiasH = 0
        self.linearEulerBiasR = 0
        self.linearEulerBiasP = 0

        self.velocityX = 0
        self.velocityY = 0
        self.velocityZ = 0
        self.velocityX_previous = 0
        self.velocityY_previous = 0
        self.velocityZ_previous = 0

        self.movVecAccelX = [0] * 32
        self.movVecAccelY = [0] * 32
        self.movVecAccelZ = [0] * 32

        self.decay_tau = 0.3
        self.decay_alpha = np.exp(-1 / (self.decay_tau * 100))

        self.current_data = {
            "velocityX": 0,
            "velocityY": 0,
            "velocityZ": 0,
            "eulerH": 0,
            "eulerR": 0,
            "eulerP": 0
        }

        self._calibrate()

    def _calibrate(self):
        self.sensor.mode = self.CONFIG_MODE
        time.sleep(0.1)
        self.sensor.mode = self.NDOF_MODE
        time.sleep(0.1)

        for _ in range(self.sample_amount):
            tmp = self.sensor.linear_acceleration
            tmp2 = self.sensor.euler
            if tmp is None or tmp2 is None:
                continue
            self.linearAccelBiasX += tmp[0]
            self.linearAccelBiasY += tmp[1]
            self.linearAccelBiasZ += tmp[2]
            self.linearEulerBiasH += tmp2[0]
            self.linearEulerBiasR += tmp2[1]
            self.linearEulerBiasP += tmp2[2]

        self.linearAccelBiasX /= self.sample_amount
        self.linearAccelBiasY /= self.sample_amount
        self.linearAccelBiasZ /= self.sample_amount
        self.linearEulerBiasH /= self.sample_amount
        self.linearEulerBiasR /= self.sample_amount
        self.linearEulerBiasP /= self.sample_amount

    def start(self):
        while True:
            tmp = self.sensor.linear_acceleration
            tmp2 = self.sensor.euler
            if tmp is None or tmp2 is None:
                continue

            linearAccelDataX = tmp[0] - self.linearAccelBiasX
            linearAccelDataY = tmp[1] - self.linearAccelBiasY
            linearAccelDataZ = tmp[2] - self.linearAccelBiasZ
            linearEulerDataH = tmp2[0] - self.linearEulerBiasH
            linearEulerDataR = tmp2[1] - self.linearEulerBiasR
            linearEulerDataP = tmp2[2] - self.linearEulerBiasP

            for i in range(31):
                self.movVecAccelX[i + 1] = self.movVecAccelX[i]
                self.movVecAccelY[i + 1] = self.movVecAccelY[i]
                self.movVecAccelZ[i + 1] = self.movVecAccelZ[i]
            self.movVecAccelX[0] = linearAccelDataX
            self.movVecAccelY[0] = linearAccelDataY
            self.movVecAccelZ[0] = linearAccelDataZ

            averageAccelX = sum(self.movVecAccelX) / 32
            averageAccelY = sum(self.movVecAccelY) / 32
            averageAccelZ = sum(self.movVecAccelZ) / 32

            self.velocityX = self.decay_alpha * self.velocityX_previous + (averageAccelX * (1 - self.decay_alpha)) / 100
            self.velocityY = self.decay_alpha * self.velocityY_previous + (averageAccelY * (1 - self.decay_alpha)) / 100
            self.velocityZ = self.decay_alpha * self.velocityZ_previous + (averageAccelZ * (1 - self.decay_alpha)) / 100

            self.velocityX_previous = self.velocityX
            self.velocityY_previous = self.velocityY
            self.velocityZ_previous = self.velocityZ

            self.current_data = {
                "velocityX": self.velocityX,
                "velocityY": self.velocityY,
                "velocityZ": self.velocityZ,
                "eulerH": linearEulerDataH,
                "eulerR": linearEulerDataR,
                "eulerP": linearEulerDataP
            }

    def get_data(self):
        return self.current_data.copy()



if __name__ == "__main__":
    observer = BNO055Observer()
    observer._calibrate()
    print("Calibration complete. Starting data collection...")
    print("Initial data:", observer.get_data())

    try:
        thread = threading.Thread(target=observer.start)
        thread.start()
        while True:
            time.sleep(0.01)
            print("Current data:", observer.get_data())
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        print("Final data:", observer.get_data())
