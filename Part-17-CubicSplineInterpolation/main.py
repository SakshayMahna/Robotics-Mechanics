from robot.trajectory import cubic_spline_interpolation
from tools.visualize import plot_joint_trajectory
import numpy as np

if __name__ == "__main__":
    q = np.array([[-2, -1, 0, 1, 2, 3],
                  [-5, 0, 1, 0, 1, 2],
                  [0, 1, 2, -1, 0, 1],
                  [5, 2, 3, -2, -1, 0],
                  [-10, 3, 4, -3, -2, -1]
                ])

    q, qd, qdd = cubic_spline_interpolation(q)
    plot_joint_trajectory(q, qd, qdd)