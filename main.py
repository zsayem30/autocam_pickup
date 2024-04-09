from roboticstoolbox import trapezoidal, mtraj, mstraj, xplot
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import *



def dVRK_traj(current, desired, timestep):
    traj = mtraj(trapezoidal, current, desired, timestep)
    return traj

def traj_with_time_stamp(traj):
    num_segments = traj.q.shape[0] - 1
    traj.t = np.zeros(num_segments)  # Initialize time for each segment
    avr_velocity = np.zeros(num_segments)  # Initialize average velocity for each segment
    denom = np.linalg.norm(
        (traj.qd[0, :] + traj.qd[1, :]) / 2)  # Initial denominator based on the first two velocity points

    for i in range(num_segments):
        traj.t[i] = np.linalg.norm(traj.q[i, :] - traj.q[i + 1, :]) /  np.linalg.norm(
            (traj.qd[i, :] + traj.qd[i + 1, :]) / 2)
    return traj

def segmented_traj(via, qdmax_input, acceleration_time, dt_value):
    traj0 = mstraj(via.T, dt=dt_value, tacc=0, qdmax=qdmax_input)
    plt.plot(traj0.q[:, 0], traj0.q[:, 1], color="red");

    traj2 = mstraj(via.T, dt=dt_value, tacc=acceleration_time, qdmax=qdmax_input)
    plt.plot(traj2.q[:, 0], traj2.q[:, 1], color="blue");
    plt.show()
    return traj2

if __name__ == '__main__':
    # trajectory = dVRK_traj([0, 2], [1, -1], 50)
    # print(trajectory.q)
    # trajectory.plot()
    # trajectory_with_time = traj_with_time_stamp(trajectory)
    # print(trajectory_with_time.t)
    # #plt.plot(trajectory_with_time.t)
    # plt.show()
    via = np.array([[1, 1, 1],
                    [23, 26, 12],
                    [36, 3, 23],
                    [4, 42, 14]])
    traj2 = segmented_traj(via.T, [3,4,5], 2, 0.2)
    print(traj2.qdd)
    # traj3 = traj_with_time_stamp(traj2)
    # print(traj3.t)