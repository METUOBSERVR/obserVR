"""
funciton file for stereo scaling functions
ObserVR 2025
Written by Ashhar Adnan
"""

import cv2
import numpy as np
from multiprocessing import Process, Pipe

"""
Constants
"""
window_size = 5
min_disp = 0
num_disp = 16 * 6
invalid_disp = 1.0
B = 12

def stereo_scaler(frameR, frameL, disp0, K1, K2, D1, D2, R, T, f, B, dzpipe, disppipe):
    h, w = frameR.shape[:2]

    # Compute rectification transforms and disparity-to-depth matrix Q
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, (w, h), R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )
    # Precompute the undistort+rectify maps
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)

    # Apply to your images
    frameL = cv2.remap(frameL, map1x, map1y, cv2.INTER_LINEAR)
    frameR = cv2.remap(frameR, map2x, map2y, cv2.INTER_LINEAR)

    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=8 * 3 * window_size ** 2,
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    disp1 = stereo.compute(frameL, frameR).astype(np.float32) / 16.0

    # mask out invalid disparities
    mask0 = disp0 > invalid_disp
    mask1 = disp1 > invalid_disp
    valid = mask0 & mask1

    # allocate output
    dz = np.zeros_like(disp1, dtype=np.float32)

    # compute depths
    Z0 = np.zeros_like(disp0, dtype=np.float32)
    Z1 = np.zeros_like(disp1, dtype=np.float32)
    Z0[valid] = (f * B) / disp0[valid]
    Z1[valid] = (f * B) / disp1[valid]

    # dz
    dz[valid] = Z0[valid] - Z1[valid]
    dz = np.mean(dz[valid])

    dzpipe.send(dz)
    disppipe.send(disp1)
