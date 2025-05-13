import cv2
import numpy as np
from pickle import load
from icecream import ic

from sys import platform

if platform == "linux" or platform == "linux2":
    from picamera2 import Picamera2
    from libcamera import controls

if platform == "linux" or platform == "linux2":
    cams = (Picamera2(0), Picamera2(1))
    for cam in cams:
        config = cam.create_video_configuration(
            controls={'FrameRate': 120, "AfMode": controls.AfModeEnum.Manual, "LensPosition": 1.1},
            main={'format': 'XRGB8888', 'size': (480, 640)},
            raw={'format': 'SRGGB10_CSI2P', 'size': (1536, 864)})
        cam.configure(config)
        cam.start("main")


def click_event(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        ic(z[y, x])


def click_event_disp(event, x, y, flags, params):
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        # displaying the coordinates
        # on the Shell
        ic(disparity[y, x])

with open("calibrationStereo_mat.calib", 'rb') as f:
    K2, D2, K1, D1, R, T = load(f)

B = np.linalg.norm(T)

ic(B)

frameR = cams[0].capture_array()
frameL = cams[1].capture_array()

frameR = cv2.rotate(frameR, cv2.ROTATE_90_CLOCKWISE)
frameL = cv2.rotate(frameL, cv2.ROTATE_90_COUNTERCLOCKWISE)

h, w = frameR.shape[:2]
ic(h, w)

# Compute rectification transforms and disparity-to-depth matrix Q
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K1, D1, K2, D2, (w, h), R, T,
    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
)

f = P1[0, 0]
ic(f)

# Precompute the undistort+rectify maps
mapLx, mapLy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_16SC2)
mapRx, mapRy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_16SC2)

def nothing(x):
    pass


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

cv2.namedWindow('disp', cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp', 600, 600)

cv2.createTrackbar('numDisparities', 'disp', numDisparities, 17, nothing)
cv2.createTrackbar('blockSize', 'disp', blockSize, 50, nothing)
cv2.createTrackbar('preFilterCap', 'disp', preFilterCap, 62, nothing)
cv2.createTrackbar('uniquenessRatio', 'disp', uniquenessRatio, 100, nothing)
cv2.createTrackbar('speckleRange', 'disp', speckleRange, 100, nothing)
cv2.createTrackbar('speckleWindowSize', 'disp', speckleWindowSize, 25, nothing)
cv2.createTrackbar('disp12MaxDiff', 'disp', disp12MaxDiff, 25, nothing)
cv2.createTrackbar('minDisparity', 'disp', minDisparity, 25, nothing)
cv2.createTrackbar('lambda', 'disp', lmbda, 100000, nothing)
cv2.createTrackbar('sigma', 'disp', sigma, 10, nothing)


while True:
    frameR = cams[0].capture_array()
    frameL = cams[1].capture_array()

    frameR = cv2.rotate(frameR, cv2.ROTATE_90_CLOCKWISE)
    frameL = cv2.rotate(frameL, cv2.ROTATE_90_COUNTERCLOCKWISE)

    imgR_gray = cv2.cvtColor(frameR,cv2.COLOR_BGR2GRAY)
    imgL_gray = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)

    Left_nice = cv2.remap(imgL_gray,
                          mapLx,
                          mapLy,
                          cv2.INTER_LANCZOS4,
                          cv2.BORDER_CONSTANT,
                          0)
    Right_nice = cv2.remap(imgR_gray,
                           mapRx,
                           mapRy,
                           cv2.INTER_LANCZOS4,
                           cv2.BORDER_CONSTANT,
                           0)

    numDisparities = cv2.getTrackbarPos('numDisparities', 'disp') * 16
    blockSize = cv2.getTrackbarPos('blockSize', 'disp')
    preFilterCap = cv2.getTrackbarPos('preFilterCap', 'disp')
    uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio', 'disp')
    speckleRange = cv2.getTrackbarPos('speckleRange', 'disp')
    speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize', 'disp')
    disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff', 'disp')
    minDisparity = cv2.getTrackbarPos('minDisparity', 'disp')
    lmbda = cv2.getTrackbarPos('lambda', 'disp')
    sigma = cv2.getTrackbarPos('sigma', 'disp')

    leftMatcher = cv2.StereoSGBM_create(
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

    rightMatcher = cv2.ximgproc.createRightMatcher(leftMatcher)

    # Calculating disparity using the StereoSGBM algorithm
    leftdisparity = leftMatcher.compute(Left_nice, Right_nice)
    rightdisparity = rightMatcher.compute(Right_nice, Left_nice)

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=leftMatcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)

    disparity = wls_filter.filter(leftdisparity, Left_nice, disparity_map_right=rightdisparity)

    disparity = (disparity.astype(np.float32))/16.0


    z = np.where(disparity > 0, (f * B) / disparity, 0)

    z[z>1000] = 1000
    z[z<0] = 0


    a = cv2.normalize(z, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    a = cv2.applyColorMap(a, cv2.COLORMAP_JET)
    dispa = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Displaying the disparity map
    cv2.imshow("disp", dispa)
    cv2.imshow("3d", a)
    cv2.setMouseCallback('3d', click_event)
    cv2.setMouseCallback('disp', click_event_disp)

    # Close window using q key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
