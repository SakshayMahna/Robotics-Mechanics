"""
Trajectory Planning Module

Point to Point Cubic
Point to Point Quintic
Point to Point Linear with Blend
"""
import numpy as np


def cubic_trajectory_planning(q0, qf, qd0, qdf, m = 100):
    """
    Point to Point Cubic Trajectory Planning

    ...

    Parameters
    ---
    q0  : Initial Position (Dof x 1)
    qf  : Final Position (Dof x 1)
    qd0 : Initial Velocity (Dof x 1)
    qf0 : Final Velocity (Dof x 1)
    m(Optional) : Discrete Time Steps

    Returns
    ---
    q, qd, qdd : Position, Velocity and Acceleration (Dof x m)
    """
    n = q0.shape[0]

    # Polynomial Parameters
    a0 = np.copy(q0)
    a1 = np.copy(qd0) 
    a2 = 3 * (qf - q0) - 2 * qd0 - qdf
    a3 = -2 * (qf - q0) + qd0 + qdf

    timesteps = np.linspace(0, 1, num = m)

    q = np.zeros((n, m))
    qd = np.zeros((n, m))
    qdd = np.zeros((n, m))

    for i in range(len(timesteps)):
        t = timesteps[i]
        t_2 = t * t
        t_3 = t * t * t

        q[:, i] = (a0) + (a1 * t) + (a2 * t_2) + (a3 * t_3)
        qd[:, i] = (a1) + (2 * a2 * t) + (3 * a3 * t_2)
        qdd[:, i] = (2 * a2) + (6 * a3 * t)

    return q, qd, qdd