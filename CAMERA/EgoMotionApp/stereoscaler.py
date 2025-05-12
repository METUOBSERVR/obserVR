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
min_disp = 10
num_disp = 16 * 6
invalid_disp = 1.0


def stereo_scaler(frameR, frameL, disp0, mapRx, mapRy, mapLx, mapLy, fR, fL, B, dzpipe, disppipe, stereo):
    """
    Stereo scaling algorithm. This function is meant to be ran using the multiprocessing module
    """

    disp1 = calc_disparity(frameR, frameL, mapRx, mapRy, mapLx, mapLy, stereo)

    # mask out invalid disparities
    mask0 = disp0 > invalid_disp
    mask1 = disp1 > invalid_disp
    valid = mask0 & mask1

    # allocate output
    dz = np.zeros_like(disp1, dtype=np.float32)

    # compute depths
    Z0 = np.zeros_like(disp0, dtype=np.float32)
    Z1 = np.zeros_like(disp1, dtype=np.float32)
    Z0[valid] = (fR * B) / disp0[valid]
    Z1[valid] = (fL * B) / disp1[valid]

    cv2.imshow("d",disp0)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # dz
    dz = Z0 - Z1
    dz = np.mean(dz[valid])

    dzpipe.send(dz)
    disppipe.send(disp1)


def calc_disparity(frameR, frameL, mapRx, mapRy, mapLx, mapLy, stereo):
    """
    This function calculated the disparity between two images
    """

    # Apply maps to your images
    frameL = cv2.remap(frameL, mapLx, mapLy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    frameR = cv2.remap(frameR, mapRx, mapRy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    disp = ((stereo.compute(frameL, frameR).astype(np.float32) / 16.0) - min_disp)/num_disp
    return disp


def compute_maps(frameR, frameL, K1, K2, D1, D2, R, T):
    h, w = frameR.shape[:2]

    # Compute rectification transforms and disparity-to-depth matrix Q
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, (w, h), R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )
    # Precompute the undistort+rectify maps
    mapLx, mapLy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    mapRx, mapRy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)

    return mapLx, mapLy, mapRx, mapRy


def create_SGBM():
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
    return stereo
