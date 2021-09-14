from robot.trajectory import linear_trajectory_planning
from tools.visualize import plot_joint_trajectory
import numpy as np

if __name__ == "__main__":
    q0 = np.array([-2, -1, 0, 1, 2, 3])
    qf = np.array([4, -3, -2, 0, 4, -2])

    q, qd, qdd = linear_trajectory_planning(q0, qf)
    plot_joint_trajectory(q, qd, qdd)