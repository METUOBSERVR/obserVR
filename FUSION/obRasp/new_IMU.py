import time
import board
import numpy as np
import adafruit_bno055
import threading
import socket
import struct

class BNO055Observer:
    CONFIG_MODE = 0x00
    NDOF_MODE = 0x0C

    def __init__(self, sample_amount=1000):
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.sample_amount = sample_amount
        self.IP         = "192.168.10.167"
        self.PORT       = 5006

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
        self.changeAxis()
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


    def changeAxis(self):
        #self.sensor.set_axis_remap(0,0,0)
        pass
        
    def start(self):
        while True:
            self.data_ready = 0
            tmp = self.sensor.linear_acceleration
            tmp2 = self.sensor.euler
            if tmp is None or tmp2 is None:
                continue
            linearAccelDataX = tmp[0] - self.linearAccelBiasX
            linearAccelDataY = tmp[1] - self.linearAccelBiasY
            linearAccelDataZ = tmp[2] - self.linearAccelBiasZ
            
            self.linearEulerDataH = tmp2[0] - self.linearEulerBiasH
            self.linearEulerDataR = tmp2[1] - self.linearEulerBiasR
            self.linearEulerDataP = tmp2[2] - self.linearEulerBiasP
            self.linearEulerDataH = ((self.linearEulerDataH + 180 ) % 360) - 180
            self.data_ready = 1
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
                "eulerH": self.linearEulerDataH,
                "eulerR": self.linearEulerDataR,
                "eulerP": self.linearEulerDataP
            }

            time.sleep(0.05)

    def get_data(self):
        return self.current_data.copy()

    def send_data_computer(self):

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((self.IP, self.PORT))
        while True:
            time.sleep(0.1)
            while not self.data_ready:
                pass
            data = struct.pack('!fff', *[self.linearEulerDataH, self.linearEulerDataP, self.linearEulerDataR])
            client_socket.send(data)


