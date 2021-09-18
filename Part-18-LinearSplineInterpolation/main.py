from robot.trajectory import linear_spline_interpolation
from tools.visualize import plot_joint_trajectory
import numpy as np

if __name__ == "__main__":
    q = np.array([[-2, 2, 0, -2],
                  [-5, 1, -1, -1],
                  [0, -1, 2, 0],
                  [2, -2, 3, 1]
                ])
    t = np.array([30, 30, 40])

    q, qd, qdd = linear_spline_interpolation(q, t)
    plot_joint_trajectory(q, qd, qdd)