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


def quintic_trajectory_planning(q0, qf, qd0, qdf, qdd0, qddf, m = 100):
    """
    Point to Point Quintic Trajectory Planning

    ...

    Parameters
    ---
    q0  : Initial Position (Dof x 1)
    qf  : Final Position (Dof x 1)
    qd0 : Initial Velocity (Dof x 1)
    qdf : Final Velocity (Dof x 1)
    qdd0 : Initial Acceleration (Dof x 1)
    qddf : Final Acceleration (Dof x 1)
    m(Optional) : Discrete Time Steps

    Returns
    ---
    q, qd, qdd : Position, Velocity and Acceleration (Dof x m)
    """
    n = q0.shape[0]

    # Polynomial Parameters
    a0 = np.copy(q0)
    a1 = np.copy(qd0) 
    a2 = np.copy(qdd0) / 2
    a3 = (20 * qf - 20 * q0 - 8 * qdf - 12 * qd0 - 3 * qdd0 + qddf) / 2
    a4 = (30 * q0 - 30 * qf + 14 * qdf + 16 * qd0 + 3 * qdd0 - 2 * qddf) / 2
    a5 = (12 * qf - 12 * q0 - 6 * qdf - 6 * qd0 - qdd0 + qddf) / 2

    timesteps = np.linspace(0, 1, num = m)

    q = np.zeros((n, m))
    qd = np.zeros((n, m))
    qdd = np.zeros((n, m))

    for i in range(len(timesteps)):
        t = timesteps[i]
        t_2 = t * t
        t_3 = t_2 * t
        t_4 = t_3 * t
        t_5 = t_4 * t

        q[:, i] = (a0) + (a1 * t) + (a2 * t_2) + (a3 * t_3) + (a4 * t_4) + (a5 * t_5)
        qd[:, i] = (a1) + (2 * a2 * t) + (3 * a3 * t_2) + (4 * a4 * t_3) + (5 * a5 * t_4)
        qdd[:, i] = (2 * a2) + (6 * a3 * t) + (12 * a4 * t_2) + (20 * a5 * t_3)

    return q, qd, qdd


def linear_trajectory_planning(q0, qf, m = 100):
    """
    Point to Point Linear Trajectory Planning with
    Parabolic Blend

    ...

    Parameters
    ---
    q0  : Initial Position (Dof x 1)
    qf  : Final Position (Dof x 1)

    Returns
    ---
    q, qd, qdd : Position, Velocity and Acceleration (Dof x m)
    """
    n = q0.shape[0]

    timesteps = np.linspace(0, 1, num = m)
    t1 = int(m * 0.1)
    t2 = int(m - m * 0.1)

    v = (qf - q0)
    a = v / timesteps[t1]

    q = np.zeros((n, m))
    qd = np.zeros((n, m))
    qdd = np.zeros((n, m))

    for i in range(len(timesteps)):
        if i < t1:
            t = timesteps[i]

            qdd[:, i] = a
            qd[:, i] = a * t
            q[:, i] = q0 + 0.5 * a * t * t
        elif i >= t1 and i < t2:
            t = timesteps[i] - timesteps[t1]

            qdd[:, i] = np.zeros(n)
            qd[:, i] = v
            q[:, i] = q[:, t1 - 1] + v * t
        else:
            t = timesteps[i] - timesteps[t2]

            qdd[:, i] = -1 * a
            qd[:, i] = v + (-1 * a * t)
            q[:, i] = q[:, t2 - 1] + v * t + (0.5 * -1 * a * t * t)

    return q, qd, qdd
