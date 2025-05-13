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
window_size = 15* 2 + 5
min_disp = 4
num_disp = 16 * 4
invalid_disp = 1.0


def stereo_scaler(frameR, frameL, disp0, mapRx, mapRy, mapLx, mapLy, f, B, Q, dzpipe, disppipe, stereoLeft, stereoRight, wls_filter):
    """
    Stereo scaling algorithm. This function is meant to be ran using the multiprocessing module
    """

    disp1 = calc_disparity(frameR, frameL, mapRx, mapRy, mapLx, mapLy, stereoLeft, stereoRight, wls_filter)

    # mask out invalid disparities
    mask0 = np.abs(disp0) > invalid_disp
    mask1 = np.abs(disp1) > invalid_disp
    valid = mask0 & mask1

    # allocate output
    dz = np.zeros_like(disp1, dtype=np.float32)

    # compute depths
    Z0 = f*B/disp0
    Z1 = f*B/disp1
    
    # dz
    dz = Z0 - Z1
    dz = np.mean(dz[valid])

    dzpipe.send(dz)
    disppipe.send(disp1)


def calc_disparity(frameR, frameL, mapRx, mapRy, mapLx, mapLy, stereoLeft, stereoRight, wls_filter):
    """
    This function calculated the disparity between two images
    """

    # Apply maps to your images
    frameL = cv2.remap(frameL, mapLx, mapLy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    frameR = cv2.remap(frameR, mapRx, mapRy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    leftdisparity = stereoLeft.compute(frameL, frameR)
    rightdisparity = stereoRight.compute(frameR, frameL)

    disp = wls_filter.filter(leftdisparity, frameL, disparity_map_right=rightdisparity)
    
    disp = ((disp.astype(np.float32) / 16.0) - min_disp) / num_disp
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

    return mapLx, mapLy, mapRx, mapRy, Q


def create_SGBM():
    stereoLeft = cv2.StereoBM_create()

    minDisparity = min_disp
    numDisparities = num_disp
    blockSize = window_size
    disp12MaxDiff = 9
    uniquenessRatio = 9
    speckleWindowSize = 14 * 2
    speckleRange = 16
    preFilterType = 0
    preFilterSize = 16 * 2 + 5
    preFilterCap = 25
    textureThreshold = 47

    stereoLeft.setNumDisparities(numDisparities)
    stereoLeft.setBlockSize(blockSize)
    stereoLeft.setPreFilterType(preFilterType)
    stereoLeft.setPreFilterSize(preFilterSize)
    stereoLeft.setPreFilterCap(preFilterCap)
    stereoLeft.setTextureThreshold(textureThreshold)
    stereoLeft.setUniquenessRatio(uniquenessRatio)
    stereoLeft.setSpeckleRange(speckleRange)
    stereoLeft.setSpeckleWindowSize(speckleWindowSize)
    stereoLeft.setDisp12MaxDiff(disp12MaxDiff)
    stereoLeft.setMinDisparity(minDisparity)

    stereoRight = cv2.ximgproc.createRightMatcher(stereoLeft)

    lmbda = 70000
    sigma = 2.0

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereoLeft)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)

    return stereoLeft, stereoRight, wls_filter
