import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from egomotion import EgoMotion
import time
import datetime
import _thread


def input_thread():
    global endFlag
    i = None
    while i != "q":
        i = input("Press q to quit: ")
    endFlag = True


if __name__ == '__main__':
    # Initialise EgoMotion
    egomotion = EgoMotion(fps=30, calibFile="calibration_logi.calib")

    tdata = []
    Rdata = []
    t = []

    # Ensure frames are in the buffer
    while egomotion.f0 is None:
        egomotion.update_frames()

    t0 = time.time()
    t.append(time.time() - t0)
    tdata.append(egomotion.current_location())
    Rdata.append(egomotion.current_rotation())

    print("Started Camera")

    endFlag = False

    _thread.start_new_thread(input_thread, ())

    # Loop until exit
    while not endFlag:
        egomotion.update_frames()
        ret = egomotion.optical_flow()

        if ret:
            egomotion.calculate_egomotion(drawpoints=False, showtR=False)

        t.append(time.time() - t0)
        tdata.append(egomotion.current_location())
        Rdata.append(egomotion.current_rotation())

    cv2.destroyAllWindows()
    egomotion.release_cam()

    tdata = np.hstack(tdata).transpose()
    Rdata = np.vstack(Rdata)
    df = pd.DataFrame(
        {"t": t, 'x': tdata[:, 0], 'y': tdata[:, 1], 'z': tdata[:, 2], 'pitch': Rdata[:, 0], 'yaw': Rdata[:, 1],
         'roll': Rdata[:, 2]})
    df.to_csv(datetime.datetime.now().strftime("outputs/%d%b%Y_%H.%M.%S.csv"), index=False)
