#!/usr/bin/env python3
import time
import numpy as np
from scipy.signal import lfilter
from EKF import ExtendedKalmanFilter
from egomotion import EgoMotion
import datetime
import cv2
import pandas as pd
import IMU
import threading
import _thread
import multiprocessing
from scipy.spatial.transform import Rotation

import socket
import struct

"""
# Setup TCP socket
TCP_IP = "10.137.77.167"   # Replace with your Windows PC IP
TCP_PORT = 9000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_IP, TCP_PORT))"""


def input_thread():
    global endFlag
    i = None
    while i != "q":
        i = input("Press q to quit: ")
    endFlag = True

# Constants
Fs = 100  # Hz
dt = 1 / Fs
decay_tau = 0.3
decay_alpha = np.exp(-1 / (decay_tau * Fs))
a = 0.75  # Complement weight for IMU velocity

# Data buffers
N = 1000

# State buffers
vx, vy, vz = 0.0, 0.0, 0.0

# initialaze egomotion
egomotion = EgoMotion(framewidth=640, frameheight=480, fps=120, calibFile="calibrationStereo.calib")

# Initialize IMU
imu = IMU.BNO055Observer()
imu._calibrate()
print('The imu is calibrated.')
threadFunctionIMU = threading.Thread(target = imu.start, daemon = True)
threadFunctionIMU.start()

# Initialize EKF
ekf = ExtendedKalmanFilter(dt=dt)

pos_cam = [np.zeros(shape=3)]
time_vector = [0]
estimated_pos = [np.zeros(shape=3)]
rotation = [np.zeros((3))]

endFlag = False

print("When the program is running, you can stop it by entering q")
i = None
while i != "Y" and i != "y":
        i = input("To start enter Y/y")
        
_thread.start_new_thread(input_thread, ())

i = 0

# Real-time loop
while not endFlag:
    t_start = time.time()
    time_vector.append((i+1)*dt)


    imu_data = imu.get_data()

    imu_velx = imu_data["velocityX"]
    imu_vely = imu_data["velocityY"]
    imu_velz = imu_data["velocityZ"]
    
    imu_yaw   = imu_data["eulerH"] 
    imu_roll  = imu_data["eulerR"]
    imu_pitch = imu_data["eulerP"]
    
    egomotion.Rpose = Rotation.from_euler( 'zxy', [imu_roll, imu_pitch, imu_yaw],degrees=True)

    rotation.append(np.array([imu_yaw, imu_roll, imu_pitch]))


    # --- Get Data ---
    egomotion.update_frames()
    ret = egomotion.optical_flow()

    if ret:
        egomotion.calculate_egomotion(drawpoints=True, showtR=False)
        
    dt = time.time() - t_start

    pos_cam.append(egomotion.current_location().reshape(3))
    vel_cam = (pos_cam[i+1] - pos_cam[i])/dt
    
    
    vel_fused = np.array([
        a * imu_velx + (1 - a) * vel_cam[0],
        a * imu_vely + (1 - a) * vel_cam[1],
        a * imu_velz + (1 - a) * vel_cam[2],
    ])

    # --- EKF ---
    ekf.predict()
    ekf.update(pos_cam[i+1], vel_fused)
    new_pos = ekf.get_state()
    estimated_pos.append(new_pos)

    egomotion.Tpose = new_pos.reshape((1,3))

    print(f"estimated:{estimated_pos[i+1]}")
    
    x, y, z = (estimated_pos[i+1][0],estimated_pos[i+1][1],estimated_pos[i+1][2])
    packet = struct.pack('dfff', t_start, x, y, z)  # d=double, fff=3 floats
    #sock.sendall(packet)
    
    i +=1
    
    # Sync loop
    #time.sleep(max(0, dt - (time.time() - t_start)))

cv2.destroyAllWindows()
egomotion.release_cam()

estimated_pos = np.array(estimated_pos)
time_vector   = np.array(time_vector)
rotation      = np.array(rotation)

df = pd.DataFrame(
        {"t": time_vector, 'x': estimated_pos[:, 0], 'y': estimated_pos[:, 1], 
         'z': estimated_pos[:, 2], 'yaw': rotation[:, 0], 'roll': rotation[:, 1],
         'pitch': rotation[:, 2]}
         )

print(df)

fname = datetime.datetime.now().strftime("outputs/%d%b%Y_%H.%M.%S.csv")
df.to_csv(fname, index=False)
print(f"Saved data at {fname}")


sock.close()


"""
# --- Plot ---
plt.figure(figsize=(18, 6))

plt.subplot(131)
plt.plot(time_vector, estimated_pos[:, 0], label='X')
plt.plot(time_vector, estimated_pos[:, 1], label='Y')
plt.plot(time_vector, estimated_pos[:, 2], label='Z')
plt.title('Estimated Position')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()

plt.subplot(132)
plt.plot(time_vector, vx * np.ones_like(time_vector), label='Vx (IMU-integrated)')
plt.plot(time_vector, vy * np.ones_like(time_vector), label='Vy')
plt.plot(time_vector, vz * np.ones_like(time_vector), label='Vz')
plt.title('Last IMU Velocity (Decay Filter)')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()

plt.subplot(133)
plt.plot(time_vector, np.linalg.norm(estimated_pos - actual_pos, axis=1))
plt.title('Position Error Norm (if actual_pos is used)')
plt.xlabel('Time [s]')
plt.ylabel('Error [m]')
plt.tight_layout()
plt.show()
"""
