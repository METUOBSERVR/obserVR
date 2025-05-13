"""
funciton file for stereo scaling functions
ObserVR 2025
Written by Ashhar Adnan
"""

import cv2
import numpy as np
from multiprocessing import Process, Pipe


def stereo_scaler(frameR, frameL, disp0, mapRx, mapRy, mapLx, mapLy, f, B, Q, dzpipe, disppipe, stereoLeft, stereoRight, wls_filter):
    """
    Stereo scaling algorithm. This function is meant to be ran using the multiprocessing module
    """

    disp1 = calc_disparity(frameR, frameL, mapRx, mapRy, mapLx, mapLy, stereoLeft, stereoRight, wls_filter)

    # mask out invalid disparities
    mask0 = disp0 > invalid_disp
    mask1 = disp1 > invalid_disp
    valid = mask0 & mask1
    

    # allocate output
    dz = np.zeros_like(disp1, dtype=np.float32)
    
    cv2.imshow("disp0",disp0)
    cv2.waitKey(0)

    # compute depths
    Z0 = cv2.reprojectImageTo3D(disp0, Q)[:,:,0]
    Z1 = cv2.reprojectImageTo3D(disp1, Q)[:,:,0]
    
    # dz
    dz = Z0 - Z1
    dz = np.mean(dz[valid])
    
    print(dz)

    dzpipe.send(dz)
    disppipe.send(disp1)


def calc_disparity(frameR, frameL, mapRx, mapRy, mapLx, mapLy, stereoLeft, stereoRight, wls_filter):
    """
    This function calculated the disparity between two images
    """

    # Apply maps to your images
    frameL = cv2.remap(frameL, mapLx, mapLy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    frameR = cv2.remap(frameR, mapRx, mapRy, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    
    dispL = stereoLeft.compute(frameL, frameR)
    dispR = stereoRight.compute(frameR, frameL)

    disp = wls_filter.filter(dispL, frameL, disparity_map_right=dispR)

    disp = (((disp).astype(np.float32) - min_disp)/ 16.0)
    return disp                                


def compute_maps(frameR, frameL, K1, K2, D1, D2, R, T):
    h, w = frameR.shape[:2]

    # Compute rectification transforms and disparity-to-depth matrix Q
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, (w, h), R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )
    # Precompute the undistort+rectify maps
    mapLx, mapLy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_16SC2)
    mapRx, mapRy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_16SC2)

    B = np.linalg.norm(T)
    f = P1[0, 0]

    return mapLx, mapLy, mapRx, mapRy, Q, B, f


def create_SGBM():
    block_size = 3
    min_disp = 5
    num_disp = 8
    invalid_disp = 0

    minDisparity = min_disp
    numDisparities = num_disp
    blockSize = block_size
    disp12MaxDiff = 25
    uniquenessRatio = 10
    speckleWindowSize = 100
    speckleRange = 32
    preFilterCap = 12

    lmbda = 70000
    sigma = 2

    stereoLeft = cv2.StereoSGBM_create(
        minDisparity=minDisparity,
        numDisparities=numDisparities,
        blockSize=blockSize,
        P1=8 * 1 * blockSize ** 2,
        P2=32 * 1 * blockSize ** 2,
        preFilterCap=preFilterCap,
        uniquenessRatio=uniquenessRatio,
        speckleWindowSize=speckleWindowSize,
        speckleRange=speckleRange,
        disp12MaxDiff=disp12MaxDiff,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    
    stereoRight = cv2.ximgproc.createRightMatcher(stereoLeft)

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereoLeft)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    
    return stereoLeft, stereoRight, wls_filter
