"""
stereo_vo.py — Frame‑in / pose‑out module (matrix‑friendly)
===========================================================
This refactor lets you create the VO object **directly from the six calibration
matrices** you already have (K₁, D₁, K₂, D₂, R, T) *or* still load them from a
YAML/JSON file.

Usage A – from matrices
----------------------
```python
import cv2 as cv, numpy as np
from stereo_vo import StereoVO

# (fill these from your calibration step)
K1 = np.array([[...], [...], [...]])
D1 = np.array([...])
K2 = np.array([[...], [...], [...]])
D2 = np.array([...])
R  = np.array([[...], [...], [...]])
T  = np.array([[...], [...], [...]])  # shape (3,1) or (3,)

vo = StereoVO(size=(640,400), K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T)

pose = vo.process(frame_L, frame_R)  # → (x, y, z) or None
```

UsageB – from YAML (unchanged)
-------------------------------
```python
vo = StereoVO(calib_path="stereo_calib.yaml", size=(640,400))
```
"""

from __future__ import annotations

import cv2 as cv
import numpy as np
from pathlib import Path
from typing import Optional, Tuple, Union
from scipy.spatial.transform import Rotation

from pickle import load
from icecream import ic

from sys import platform

if platform == "linux" or platform == "linux2":
    from picamera2 import Picamera2
    from libcamera import controls

Array = np.ndarray


# ---- SGBM + WLS parameters --------------------------------------------------
_BLOCK_SIZE         = 3
_MIN_DISP           = 5
_NUM_DISP           = 8                # user value (will be ×16 to satisfy OpenCV)
_DISP12_MAX_DIFF    = 25
_UNIQUENESS_RATIO   = 10
_SPECKLE_WINDOW     = 100
_SPECKLE_RANGE      = 32
_PREFILTER_CAP      = 12

_WLS_LAMBDA         = 70_000
_WLS_SIGMA_COLOR    = 2

# Ensure numDisparities is multiple of 16 as required by OpenCV
_NUM_DISP16 = int(np.ceil(_NUM_DISP / 16)) * 16


class StereoVO:
    """Stateful stereo visual odometry with external orientation override."""

    # ------------------------------------------------------------------
    def __init__(
        self,
        size: Tuple[int, int] = (640, 400),
        *,
        calib_path: Union[str, Path, None] = None,
        K1: Optional[Array] = None,
        D1: Optional[Array] = None,
        K2: Optional[Array] = None,
        D2: Optional[Array] = None,
        R:  Optional[Array] = None,
        T:  Optional[Array] = None,
        nfeatures: int = 1000,
    ) -> None:
        self.size = tuple(size)

        # ---- Load calibration ------------------------------------
        if calib_path is not None:
            K1, D1, K2, D2, R, T = self._load_calib_file(calib_path)
        elif None in (K1, D1, K2, D2, R, T):
            raise ValueError("Provide calib_path OR all six matrices")
        K1, D1, K2, D2, R, T = [np.asarray(m, dtype=np.float64) for m in (K1, D1, K2, D2, R, T)]
        T = T.reshape(3, 1)
        self._K1 = K1

        # ---- Rectification ---------------------------------------
        R1, R2, P1, P2, Q, _, _ = cv.stereoRectify(
            K1, D1, K2, D2, self.size, R, T,
            flags=cv.CALIB_ZERO_DISPARITY, alpha=0)
        self._Q = Q
        self._map1x, self._map1y = cv.initUndistortRectifyMap(
            K1, D1, R1, P1, self.size, cv.CV_32FC1)
        self._map2x, self._map2y = cv.initUndistortRectifyMap(
            K2, D2, R2, P2, self.size, cv.CV_32FC1)

        # ---- StereoSGBM + WLS ------------------------------------
        self._sgbm_left = cv.StereoSGBM_create(
            minDisparity=_MIN_DISP,
            numDisparities=_NUM_DISP16,
            blockSize=_BLOCK_SIZE,
            P1=8 * 3 * _BLOCK_SIZE**2,
            P2=32 * 3 * _BLOCK_SIZE**2,
            preFilterCap=_PREFILTER_CAP,
            uniquenessRatio=_UNIQUENESS_RATIO,
            speckleWindowSize=_SPECKLE_WINDOW,
            speckleRange=_SPECKLE_RANGE,
            disp12MaxDiff=_DISP12_MAX_DIFF,
            mode=cv.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        # Right matcher shares most parameters
        self._sgbm_right = cv.ximgproc.createRightMatcher(self._sgbm_left)

        self._wls = cv.ximgproc.createDisparityWLSFilter(matcher_left=self._sgbm_left)
        self._wls.setLambda(_WLS_LAMBDA)
        self._wls.setSigmaColor(_WLS_SIGMA_COLOR)

        # ---- ORB + matcher ---------------------------------------
        self._orb = cv.ORB_create(nfeatures=nfeatures, fastThreshold=10)
        self._bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        # ---- Pose history ----------------------------------------
        self._pose = np.eye(4, dtype=np.float32)
        self._prev_pts3d: Optional[Array] = None
        self._prev_desc: Optional[Array] = None
        self._prev_kp: Optional[list[cv.KeyPoint]] = None

    # ------------------------------------------------------------------
    @staticmethod
    def _load_calib_file(path: Union[str, Path]):
        with open(path, 'rb') as f:
            K2, D2, K1, D1, R, T = load(f)
        return K1, D1, K2, D2, R, T

    # ------------------------------------------------------------------
    def reset_origin(self):
        self._pose = np.eye(4, dtype=np.float32)

    # ------------------------------------------------------------------
    def process(
        self,
        frame_left: Array,
        frame_right: Array,
        *,
        ext_rotation: Optional[Rotation] = None,
    ) -> Optional[Tuple[float, float, float]]:
        grayL = frame_left if frame_left.ndim == 2 else cv.cvtColor(frame_left, cv.COLOR_BGR2GRAY)
        grayR = frame_right if frame_right.ndim == 2 else cv.cvtColor(frame_right, cv.COLOR_BGR2GRAY)

        # ---- Rectify ------------------------------------------------
        lrect = cv.remap(grayL, self._map1x, self._map1y, cv.INTER_LINEAR)
        rrect = cv.remap(grayR, self._map2x, self._map2y, cv.INTER_LINEAR)

        # ---- Disparity (left+right) --------------------------------
        disp_left  = self._sgbm_left.compute(lrect, rrect)
        disp_right = self._sgbm_right.compute(rrect, lrect)

        # ---- WLS filtering ----------------------------------------
        disp_filt = self._wls.filter(disp_left, lrect, disparity_map_right=disp_right)
        disp_f = disp_filt.astype(np.float32) / 16.0  # cv returns 16× value

        pts3d = cv.reprojectImageTo3D(disp_f, self._Q)

        # ---- Features & VO logic (unchanged) ----------------------
        kp, desc = self._orb.detectAndCompute(lrect, None)
        if desc is None or len(kp) < 8:
            return None
        kp3d = np.array([pts3d[int(k.pt[1]), int(k.pt[0])] for k in kp], dtype=np.float32)

        if self._prev_desc is None:
            self._prev_pts3d, self._prev_desc, self._prev_kp = kp3d, desc, kp
            if ext_rotation is not None:
                self._pose[:3, :3] = ext_rotation.as_matrix().astype(np.float32)
            return None

        matches = self._bf.match(self._prev_desc, desc)
        if len(matches) < 8:
            self._prev_pts3d, self._prev_desc, self._prev_kp = kp3d, desc, kp
            return None
        idx_prev = [m.queryIdx for m in matches]; idx_curr = [m.trainIdx for m in matches]
        obj_pts  = self._prev_pts3d[idx_prev]
        img_pts  = np.float32([kp[i].pt for i in idx_curr])
        valid    = np.isfinite(obj_pts).all(axis=1)
        obj_pts, img_pts = obj_pts[valid], img_pts[valid]
        if obj_pts.shape[0] < 6:
            self._prev_pts3d, self._prev_desc, self._prev_kp = kp3d, desc, kp
            return None

        ok, rvec, tvec, _ = cv.solvePnPRansac(
            obj_pts, img_pts, self._K1, None,
            reprojectionError=3.0, flags=cv.SOLVEPNP_ITERATIVE)
        if ok:
            R_rel, _ = cv.Rodrigues(rvec)
            T_rel = np.eye(4, dtype=np.float32)
            T_rel[:3, :3] = R_rel.astype(np.float32)
            T_rel[:3, 3]  = tvec[:, 0].astype(np.float32)
            self._pose = self._pose @ T_rel

        if ext_rotation is not None:
            self._pose[:3, :3] = ext_rotation.as_matrix().astype(np.float32)

        self._prev_pts3d, self._prev_desc, self._prev_kp = kp3d, desc, kp

        if ok:
            return tuple(map(float, self._pose[:3, 3]))
        return None

# ------------------------------------------------------------------
if __name__ == "__main__":
    if platform == "linux" or platform == "linux2":
        cams = (Picamera2(0), Picamera2(1))
        for cam in cams:
            config = cam.create_video_configuration(
                controls={'FrameRate': 120, "AfMode": controls.AfModeEnum.Manual, "LensPosition": 1.1},
                main={'format': 'XRGB8888', 'size': (480, 640)},
                raw={'format': 'SRGGB10_CSI2P', 'size': (1536, 864)})
            cam.configure(config)
            cam.start('main')

    vo = StereoVO((480,640), calib_path="calibrationStereo_mat.calib")

    while True:
        frameR = cams[0].capture_array()
        frameL = cams[1].capture_array()

        frameR = cv.rotate(frameR, cv.ROTATE_90_CLOCKWISE)
        frameL = cv.rotate(frameL, cv.ROTATE_90_COUNTERCLOCKWISE)

        pose = vo.process(frameL, frameR)

        print(pose)
