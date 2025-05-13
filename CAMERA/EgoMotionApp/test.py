import cv2
import numpy as np
from pickle import load

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
        print(z[y,x])
        
def click_event_disp(event, x, y, flags, params): 
  
    # checking for left mouse clicks 
    if event == cv2.EVENT_LBUTTONDOWN: 
  
        # displaying the coordinates 
        # on the Shell 
        print(disparity[y,x])
        

frameR = cams[0].capture_array()
frameL = cams[1].capture_array()

frameR = cv2.rotate(frameR, cv2.ROTATE_90_CLOCKWISE)
frameL = cv2.rotate(frameL, cv2.ROTATE_90_COUNTERCLOCKWISE)

with open("calibrationStereo.calib", 'rb') as f:
    K2, D2, K1, D1, R, T = load(f)
    
f = np.mean([K2[0,0], K2[1,1], K1[0,0], K1[1,1]])
B = np.linalg.norm(T)

print(B)

h, w = frameR.shape[:2]
print(h,w)

# Compute rectification transforms and disparity-to-depth matrix Q
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K1, D1, K2, D2, (w, h), R, T,
    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
)
# Precompute the undistort+rectify maps
mapLx, mapLy = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
mapRx, mapRy = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)

def nothing(x):
    pass
    
window_size = 3
min_disp = 2
num_disp = 7
invalid_disp = 0

minDisparity = min_disp
numDisparities = num_disp
blockSize = window_size
disp12MaxDiff = 14
uniquenessRatio = 22
speckleWindowSize = 20
speckleRange = 65
preFilterType = 0
preFilterSize = 23
preFilterCap = 54
textureThreshold = 86


cv2.namedWindow('disp', cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp', 600, 600)

cv2.createTrackbar('numDisparities', 'disp', numDisparities, 17, nothing)
cv2.createTrackbar('blockSize', 'disp', blockSize, 50, nothing)
cv2.createTrackbar('preFilterType', 'disp', preFilterType, 1, nothing)
cv2.createTrackbar('preFilterSize', 'disp', preFilterSize, 25, nothing)
cv2.createTrackbar('preFilterCap', 'disp', preFilterCap, 62, nothing)
cv2.createTrackbar('textureThreshold', 'disp', textureThreshold, 100, nothing)
cv2.createTrackbar('uniquenessRatio', 'disp', uniquenessRatio, 100, nothing)
cv2.createTrackbar('speckleRange', 'disp', speckleRange, 100, nothing)
cv2.createTrackbar('speckleWindowSize', 'disp', speckleWindowSize, 25, nothing)
cv2.createTrackbar('disp12MaxDiff', 'disp', disp12MaxDiff, 25, nothing)
cv2.createTrackbar('minDisparity', 'disp', minDisparity, 25, nothing)

# Creating an object of StereoBM algorithm
leftMatcher = cv2.StereoBM.create()


while True:
    frameL = cams[1].capture_array()
    frameR = cams[0].capture_array()
    
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

    # Updating the parameters based on the trackbar positions
    numDisparities = cv2.getTrackbarPos('numDisparities', 'disp') * 16
    blockSize = cv2.getTrackbarPos('blockSize', 'disp') * 2 + 5
    preFilterType = cv2.getTrackbarPos('preFilterType', 'disp')
    preFilterSize = cv2.getTrackbarPos('preFilterSize', 'disp') * 2 + 5
    preFilterCap = cv2.getTrackbarPos('preFilterCap', 'disp')
    textureThreshold = cv2.getTrackbarPos('textureThreshold', 'disp')
    uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio', 'disp')
    speckleRange = cv2.getTrackbarPos('speckleRange', 'disp')
    speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize', 'disp') * 2
    disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff', 'disp')
    minDisparity = cv2.getTrackbarPos('minDisparity', 'disp')

    # Setting the updated parameters before computing disparity map
    leftMatcher.setNumDisparities(numDisparities)
    leftMatcher.setBlockSize(blockSize)
    leftMatcher.setPreFilterType(preFilterType)
    leftMatcher.setPreFilterSize(preFilterSize)
    leftMatcher.setPreFilterCap(preFilterCap)
    leftMatcher.setTextureThreshold(textureThreshold)
    leftMatcher.setUniquenessRatio(uniquenessRatio)
    leftMatcher.setSpeckleRange(speckleRange)
    leftMatcher.setSpeckleWindowSize(speckleWindowSize)
    leftMatcher.setDisp12MaxDiff(disp12MaxDiff)
    leftMatcher.setMinDisparity(minDisparity)

    rightMatcher = cv2.ximgproc.createRightMatcher(leftMatcher)


    # Calculating disparity using the StereoBM algorithm
    leftdisparity = leftMatcher.compute(Left_nice, Right_nice)
    rightdisparity = rightMatcher.compute(Right_nice,Left_nice)

    lmbda = 70000
    sigma = 2

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=leftMatcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    
    
    

    disparity = wls_filter.filter(leftdisparity, Left_nice, disparity_map_right=rightdisparity)

    # Converting to float32
    disparity = disparity.astype(np.float32)

    # Scaling down the disparity values and normalising them
    disparity = ((disparity - minDisparity)/ 16.0)
    
    disparity[disparity<1] = 1
    
    
    z = (f*B)/(disparity*1e-6)
    
    #z[z>500] = 500
    #z[z<0] = 0
    
    #z = cv2.ximgproc.guidedFilter(guide = Left_nice, src=z, radius=10, eps=1e-2)
    
    a = cv2.normalize(z, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    dispa = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Displaying the disparity map
    cv2.imshow("disp", dispa)
    cv2.imshow("3d", a)
    cv2.setMouseCallback('3d', click_event) 
    cv2.setMouseCallback('disp', click_event_disp) 
    


    # Close window using q key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
