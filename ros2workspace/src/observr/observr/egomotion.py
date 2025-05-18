"""
Class file for EgoMotion
ObserVR 2024
Written by Ashhar Adnan
"""

import cv2
import numpy as np
from copy import deepcopy
from scipy.spatial.transform import Rotation
from pickle import dump, load
from sys import platform
import os
from ament_index_python.packages import get_package_share_directory


class EgoMotion:
    """
    EgoMotion class

    ----Class Variables----
    Rpose: Current cumulative rotation quarternion from origin
    Tpose: Current translation vector from origin

    cam: Camera object
    mtx: Camera calibration matrix
    dist: Camera distortion coefficients

    f0: Previous frame (GREY)
    f1: Current frame (GREY)
    frame: Current frame (RGB)

    fc0: Previous frame Complimentary (GREY)
    fc1: Current frame Complimentary (GREY)
    framec: Current frame Complimentary (RGB)

    lk_params: Optical Flow parameters
    fast: Fast feature detector object

    good_p0: Previous good keypoints
    good_p1: Current good keypoints

    ----Class Methods----
    update_frames(): Updates the current and previous frames
    optical_flow(): Optical flow calculation
    calculate_egomotion(): Calculates ego motion
    calculate_scaling(): Calculates the scaling factor through stereo disparity
    release_cam(): Releases current camera
    init_camera(): Initializes camera object
    load_calibration(): Loads calibration
    current_location(): Returns current location
    current_rotation(): Return current rotation
    """

    def __init__(self, t=np.zeros(shape=(3)), r=np.eye(3),
                 capdev=0, framewidth=640, frameheight=480, fps=30, calibFile="calibration.calib"):
        """
        EgoMotion constructor
        :param t: initial translation vector
        :param r: initial rotation quarternion
        :param capdev: capture device
        :param framewidth: frame width
        :param frameheight: frame height
        :param fps: fps
        :param calibFile: path to calibration file
        """
        self.Rpose = Rotation.from_matrix(r)
        self.Tpose = t

        share_dir = get_package_share_directory('observr')
        print(share_dir)
        config_dir = os.path.join(share_dir)
        calib_path = os.path.join(config_dir, calibFile)
        self.load_calibration(calib_path)

        self.f0 = None
        self.f1 = None
        self.frame = None
        
        # Ensure frames are in the buffer
        while self.f0 is None:
            self.update_frames(np.zeros((480, 640, 3), dtype=np.uint8))

        self.good_p0 = np.empty((0, 2), dtype=np.float32)
        self.good_p1 = np.empty((0, 2), dtype=np.float32)

        self.lk_params = dict(winSize=(5, 5),
                              maxLevel=10,
                              criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)

    def update_frames(self, frame):
        """
        Updates current and previous frames
        :return: None
        """
        self.f0 = deepcopy(self.f1)

        self.frame = frame

        self.f1 = cv2.cvtColor(self.frame, cv2.COLOR_RGB2GRAY)

    def optical_flow(self):
        """
        One step of optical flow calculation
        Detects features from previous frame and updates current frame if needed
        :return: True if features are detected else False
        """
        
        p0 = self.fast.detect(self.f0)
        p0 = np.array([x.pt for x in p0], dtype=np.float32).reshape(-1, 1, 2)

        p1, st, err = cv2.calcOpticalFlowPyrLK(self.f0, self.f1, p0, None, **self.lk_params)

        if p1 is not None:
            self.good_p0 = p0[st == 1]
            self.good_p1 = p1[st == 1]
        else:
            self.good_p0 = np.empty((0, 2), dtype=np.float32)

        return True if self.good_p0.size != 0 else False

    def calculate_egomotion(self, drawpoints=False, showtR=False):
        """
        Calculates egomotion based on current frame and previous frame using Essential Matrix decomposition
        :param drawpoints: Whether to draw points on preview frame
        :param showtR: Whether to show translation and rotation on preview frame
        :return: None
        """

        E, _ = cv2.findEssentialMat(self.good_p1, self.good_p0, self.mtx, cv2.RANSAC, 0.999, 1.0, None)
        if E is not None:
            if not np.isnan(E).any() and E.size == 9:
                _, R, t, _ = cv2.recoverPose(E, self.good_p0, self.good_p1, self.mtx)

                R = Rotation.from_matrix(R)
                t = np.reshape(t, (3))
            else:
                t=np.zeros(shape=(3))
                R = Rotation.from_matrix(np.eye(3))
        else:
            t=np.zeros(shape=(3))
            R = Rotation.from_matrix(np.eye(3))

        Rmag = abs(R.as_euler('xyz', degrees=True))
        if Rmag.max() < 20:  # Rotation Threshold
            if abs(t).max() > 0.5:
                self.Tpose = self.Tpose + self.Rpose.inv().apply(t)
            self.Rpose = R * self.Rpose

        if drawpoints:
            for i in range(len(self.good_p0)):
                cv2.circle(self.frame, (int(self.good_p0[i][0]), int(self.good_p0[i][1])), 1, (255, 0, 0))
                cv2.circle(self.frame, (int(self.good_p1[i][0]), int(self.good_p1[i][1])), 1, (0, 0, 255))

        if showtR:
            cv2.putText(self.frame, "PITCH{0:.4f} YAW{1:.4f} ROLL{2:.4f}".format(*self.Rpose.as_euler('xyz', degrees=True)), (100, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
            cv2.putText(self.frame, "X{0:.4f} Y{1:.4f} Z{2:.4f}".format(*self.Tpose.flatten()), (100, 200), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))

    def current_location(self):
        """
        Returns current location
        :return: array[x, y, z]
        """
        return self.Tpose

    def current_rotation(self, asMatrix=False):
        """
        Returns current rotation
        :param asMatrix: return matrix or not
        :return: Rotation matrix or rotation angles
        """
        if asMatrix:
            return self.Rpose.as_matrix()
        else:
            return self.Rpose.as_quat()

    def load_calibration(self, calibFile="calibration.calib"):
        """
        Loads camera calibration parameters
        :param calibFile: Path to calibration file
        :return: None
        """

        with open(calibFile, 'rb') as f:
            mtx, dist = load(f)

        self.mtx = mtx
        self.dist = dist
        
