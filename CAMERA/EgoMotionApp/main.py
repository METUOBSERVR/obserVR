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
    egomotion = EgoMotion(framewidth=640, frameheight=480, fps=120, calibFile="calibration_logi.calib")
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

    print("When the program is running, you can stop it by entering q")
    i = None
    while i != "Y" and i != "y":
        i = input("To start enter Y/y")

    _thread.start_new_thread(input_thread, ())

    # Loop until exit
    while not endFlag:
        egomotion.update_frames()
        ret = egomotion.optical_flow()

        if ret:
            egomotion.calculate_egomotion(drawpoints=True, showtR=False)

        td = egomotion.current_location()
        Rd = egomotion.current_rotation()

        t.append(time.time() - t0)
        tdata.append(td)
        Rdata.append(Rd)
        cv2.imshow('R', egomotion.frame)
        cv2.waitKey(1)
        print(f"X\t{td[0][0]:.4f}\t\tY\t{td[1][0]:.4f}\t\tZ\t{td[2][0]:.4f}\tPIT\t{Rd[0]:.4f}\t\tYAW\t{Rd[1]:.4f}\t\tROL\t{Rd[2]:.4f}")

    cv2.destroyAllWindows()
    egomotion.release_cam()

    tdata = np.hstack(tdata).transpose()
    Rdata = np.vstack(Rdata)
    df = pd.DataFrame(
        {"t": t, 'x': tdata[:, 0], 'y': tdata[:, 1], 'z': tdata[:, 2], 'pitch': Rdata[:, 0], 'yaw': Rdata[:, 1],
         'roll': Rdata[:, 2]})
    fname = datetime.datetime.now().strftime("outputs/%d%b%Y_%H.%M.%S.csv")
    df.to_csv(fname, index=False)
    print(f"Saved data at {fname}")
