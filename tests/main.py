import sys
import matplotlib.pyplot as plt
import os

if __name__ == "__main__":
    stdin_arr = []
    est_px = []
    est_py = []
    est_vx = []
    est_vy = []
    meas_px = []
    meas_py = []
    gt_px = []
    gt_py = []
    gt_vx = []
    gt_vy = []

    for line in sys.stdin:
        
        stdin_arr = line.split(",")

        est_px.append(float(stdin_arr[0]))
        est_py.append(float(stdin_arr[1]))
        est_vx.append(float(stdin_arr[2]))
        est_vy.append(float(stdin_arr[3]))
        meas_px.append(float(stdin_arr[4]))
        meas_py.append(float(stdin_arr[5]))
        gt_px.append(float(stdin_arr[6]))
        gt_py.append(float(stdin_arr[7]))
        gt_vx.append(float(stdin_arr[8]))
        gt_vy.append(float(stdin_arr[9].strip("\n")))



    plt.scatter(meas_px, meas_py, color="g", marker="x", zorder=2)
    plt.plot(gt_px, gt_py, color="b", zorder=1)
    plt.plot(est_px, est_py, color="r", zorder=3)
    plt.xlabel("px")
    plt.ylabel("py")
    plt.legend(["ground truth","EKF estimation","measurement"])
    plt.savefig("plots/ekf-1.png")
