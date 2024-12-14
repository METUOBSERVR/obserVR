import cv2
import numpy as np
from egomotion import EgoMotion

if __name__ == '__main__':
    # Initialise EgoMotion
    egomotion = EgoMotion(fps=30, calibFile="calibration_logi.calib")

    # Ensure frames are in the buffer
    while egomotion.f0 is None:
        egomotion.update_frames()

    # Loop until exit
    while True:
        egomotion.update_frames()
        ret = egomotion.optical_flow()

        if ret:
            egomotion.calculate_egomotion()

        cv2.imshow('EgoMotion', egomotion.frame)
        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    egomotion.release_cam()
