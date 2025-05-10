import cv2
import numpy as np
from icecream import ic
from copy import deepcopy
from picamera2 import Picamera2
from libcamera import controls

CHECKERBOARD = (10, 7)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cooldown_time = 35

# Open the cameras
cams = (Picamera2(0), Picamera2(1))
for cam in cams:
    config = cam.create_video_configuration(
        controls={'FrameRate': 120, "AfMode": controls.AfModeEnum.Manual, "LensPosition": 1.1},
        main={'format': 'XRGB8888', 'size': (480, 640)},
        raw={'format': 'SRGGB10_CSI2P', 'size': (1536, 864)})
    cam.configure(config)
    cam.start("main")

frame_width = 480
frame_height = 640
fps = 120

ic(frame_width, frame_height, fps)

n = 0

while True:
    rawR = cams[0].capture_array()
    rawL = cams[1].capture_array()

    rawR = cv2.rotate(rawR, cv2.ROTATE_90_CLOCKWISE)
    rawL = cv2.rotate(rawL, cv2.ROTATE_90_COUNTERCLOCKWISE)

    grayR = cv2.cvtColor(rawR, cv2.COLOR_RGB2GRAY)
    grayL = cv2.cvtColor(rawL, cv2.COLOR_RGB2GRAY)

    frameR = deepcopy(rawR)
    frameL = deepcopy(rawL)
    
    frame = np.concatenate((frameR, frameL),  axis=1)

    retR, cornersR = cv2.findChessboardCorners(grayR, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    retL, cornersL = cv2.findChessboardCorners(grayL, CHECKERBOARD,
                                               cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if retR and retL:
        # refining pixel coordinates for given 2d points.
        corners2R = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
        corners2L = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)

        # Draw and display the corners
        frameR = cv2.drawChessboardCorners(frameR, CHECKERBOARD, corners2R, retR)
        frameL = cv2.drawChessboardCorners(frameL, CHECKERBOARD, corners2L, retR)

        frame = np.concatenate((frameR, frameL),  axis=1)

    if retR and retL:
        cooldown -= 1
        cv2.putText(frame, "Cooldown: " + str(cooldown), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
        cv2.putText(frame, "Num frames: " + str(n), (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),
                   1)

        if cooldown <= 0:
                ic(n)
                cv2.imwrite(f'calibration images/picamv3/STEREO/Right{n}.png'.format(n), rawR)
                cv2.imwrite(f'calibration images/picamv3/STEREO/Left{n}.png'.format(n), rawL)
                ic(n)
                n += 1
                cv2.destroyAllWindows()
                cooldown = cooldown_time
    else:
        cooldown = cooldown_time

    cv2.imshow('concat img', frame)
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

# Release the capture and writer objects
cams[0].stop()
cams[1].stop()
