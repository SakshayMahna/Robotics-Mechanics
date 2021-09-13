from robot.trajectory import quintic_trajectory_planning
from tools.visualize import plot_joint_trajectory
import numpy as np

if __name__ == "__main__":
    q0 = np.array([-2, -1, 0, 1, 2, 3])
    qd0 = np.array([0, 0, 0, 0, 0, 0])
    qdd0 = np.array([0, 0, 0, 0, 0, 0])
    qf = np.array([4, -3, -2, 0, 4, -2])
    qdf = np.array([0, 0, 0, 0, 0, 0])
    qddf = np.array([0, 0, 0, 0, 0, 0])

    q, qd, qdd = quintic_trajectory_planning(q0, qf, qd0, qdf, qdd0, qddf)
    plot_joint_trajectory(q, qd, qdd)