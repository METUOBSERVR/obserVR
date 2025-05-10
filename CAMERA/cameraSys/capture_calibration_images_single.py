import cv2
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

frame_width = 480
frame_height = 640
fps = 120

ic(frame_width, frame_height, fps)

# Right Cam
cams[0].start("main")
n = 0

cooldown = cooldown_time

while True:
    raw = cams[0].capture_array()
    raw = cv2.rotate(raw, cv2.ROTATE_90_CLOCKWISE)
    gray = cv2.cvtColor(raw, cv2.COLOR_RGB2GRAY)
    frame = deepcopy(raw)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Draw and display the corners
        frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

    if ret:
        cooldown -= 1
        cv2.putText(frame, "Cooldown: " + str(cooldown), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
        cv2.putText(frame, "Num frames: " + str(n), (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),
                   1)

        if cooldown <= 0:
                ic(n)
                cv2.imwrite(f'calibration images/picamv3/Right{n}.png'.format(n), raw)
                ic(n)
                n += 1
                cv2.destroyAllWindows()
                cooldown = cooldown_time
    else:
        cooldown = cooldown_time

    cv2.imshow('Right img', frame)
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

# Release the capture and writer objects
cams[0].stop()


# left Cam
cams[1].start("main")
n = 0

cooldown = cooldown_time

while True:
    raw = cams[1].capture_array()
    raw = cv2.rotate(raw, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(raw, cv2.COLOR_RGB2GRAY)
    frame = deepcopy(raw)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Draw and display the corners
        frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)

    if ret:
        cooldown -= 1
        cv2.putText(frame, "Cooldown: " + str(cooldown), (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
        cv2.putText(frame, "Num frames: " + str(n), (50, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0),
                    1)

        if cooldown <= 0:
            ic(n)
            cv2.imwrite(f'calibration images/picamv3/Left{n}.png'.format(n), raw)
            ic(n)
            n += 1
            cv2.destroyAllWindows()
            cooldown = cooldown_time
    else:
        cooldown = cooldown_time

    cv2.imshow('Right img', frame)
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break

# Release the capture and writer objects
cams[1].stop()
