"""
Trajectory Planning Module

Point to Point Cubic
Point to Point Quintic
Point to Point Linear with Blend

Cubic Spline Interpolation
Linear Spline Interpolation
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
    m(Optional) : Discrete Time Steps

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


def cubic_spline_interpolation(q_, m = 100):
    """
    Cubic Spline Interpolation

    ...

    Parameters
    ---
    q_  : Array of Position (n x Dof)
    m(Optional) : Discrete Time Steps

    Returns
    ---
    q, qd, qdd : Position, Velocity and Acceleration (Dof x m)
    """
    n = q_.shape[0]
    dof = q_.shape[1]

    q_ = np.transpose(q_)

    m = m + (m % (n-1))
    k = int(m / (n-1))
    timesteps = [np.linspace(0, 1, num = k, endpoint = False) for _ in range(n-2)]
    timesteps.append(np.linspace(0, 1, num = k))

    # Generate A matrix
    A = np.zeros((dof, n, n))
    # A[:, 0, 0] = 2
    # A[:, 0, 1] = 1
    # A[:, n-1, n-2] = 1
    # A[:, n-1, n-1] = 2
    A[:, 0, 0] = 1
    A[:, n-1, n-1] = 1
    for i in range(1, n-1):
        A[:, i, i - 1] = 1
        A[:, i, i] = 4
        A[:, i, i + 1] = 1

    # Generate b matrix
    y = np.zeros((dof, n))
    # y[:, 0] = 3 * (q_[:, 1] - q_[:, 0])
    # y[:, n-1] = 3 * (q_[:, n - 1] - q_[:, n - 2])
    y[:, 0] = 0
    y[:, n-1] = 0
    for i in range(1, n-1):
        y[:, i] = 3 * (q_[:, i + 1] - q_[:, i - 1])

    # Solve D
    D = np.linalg.solve(A, y)

    # Calculate coefficients
    a = np.copy(q_[:, :n-1])
    b = np.copy(D[:, :n-1])
    c = np.zeros((dof, n-1))
    d = np.zeros((dof, n-1))
    for i in range(0, n-1):
        c[:, i] = 3 * (q_[:, i + 1] - q_[:, i]) - 2 * D[:, i] - D[:, i + 1]
        d[:, i] = 2 * (q_[:, i] - q_[:, i + 1]) + D[:, i] + D[:, i + 1]

    
    # Calculate Trajectories
    q = np.zeros((dof, m))
    qd = np.zeros((dof, m))
    qdd = np.zeros((dof, m))

    for j in range(n - 1):
        for i in range(len(timesteps[j])):
            t = timesteps[j][i]
            t_2 = t * t
            t_3 = t * t * t

            q[:, i + j * k] = a[:, j] + b[:, j] * t + c[:, j] * t_2 + d[:, j] * t_3
            qd[:, i + j * k] = b[:, j] + 2 * c[:, j] * t + 3 * d[:, j] * t_2
            qdd[:, i + j * k] = 2 * c[:, j] + 6 * d[:, j] * t

    return q, qd, qdd

def linear_spline_interpolation(q_, t_, m = 100):
    """
    Linear Spline Interpolation

    ...

    Parameters
    ---
    q_  : Array of Position (n x Dof)
    t_  : Array of Time Segments (n - 1)
    m(Optional) : Discrete Time Steps

    Returns
    ---
    q, qd, qdd : Position, Velocity and Acceleration (Dof x m)
    """
    n = q_.shape[0]
    dof = q_.shape[1]

    t_ = t_ / m
    timesteps = np.linspace(0, 1, num = m)

    a = 100
    time_segments = np.zeros((n, dof))
    blend_times = np.zeros((n, dof))
    velocities = np.zeros((n, dof))
    accelerations = np.zeros((n, dof))

    # Initial time segment
    accelerations[0] = np.sign(q_[1] - q_[0]) * a
    blend_times[0] = t_[0] - np.sqrt(
                     t_[0] * t_[0] - 2 * (q_[1] - q_[0]) / accelerations[0])
    velocities[0] = (q_[1] - q_[0]) / (t_[0] - 0.5 * blend_times[0])

    # Final time segment
    accelerations[n - 1] = np.sign(q_[n - 2] - q_[n - 1]) * a
    blend_times[n - 1] = t_[n - 2] - np.sqrt(
                         t_[n - 2] * t_[n - 2] + 2 * (q_[n - 1] - q_[n - 2]) / accelerations[n - 1])
    velocities[n - 2] = (q_[n - 1] - q_[n - 2]) / (t_[n - 2] - 0.5 * blend_times[n - 1])
    velocities[n - 1] = 0

    # Loop for velocities
    for i in range(1, n - 2):
        velocities[i] = (q_[i + 1] - q_[i]) / t_[i]

    # Loop for accelerations and blend times
    for i in range(1, n - 1):
        accelerations[i] = np.sign(velocities[i] - velocities[i - 1]) * a
        blend_times[i] = (velocities[i] - velocities[i - 1]) / accelerations[i]

    # Calculate time segments
    time_segments[0] = t_[0] - blend_times[0] - 0.5 * blend_times[1]
    time_segments[n - 2] = t_[n - 2] - blend_times[n - 1] - 0.5 * blend_times[n - 2]
    time_segments[n - 1] = 0
    for i in range(1, n - 2):
        time_segments[i] = t_[i] - 0.5 * blend_times[i + 1] - 0.5 * blend_times[i]

    
    # Calculate Trajectories
    q = np.zeros((dof, m))
    qd = np.zeros((dof, m))
    qdd = np.zeros((dof, m))

    # Loop for each degree of freedom
    for d in range(dof):
        # j for using above parameters
        # previous_i for saving i of start of a parabola segment
        # previous_ii for saving i of start of a linear segment
        j = 0
        previous_i = 0
        previous_ii = 0

        # Loop over the timesteps
        for i in range(len(timesteps)):
            t = timesteps[i] - timesteps[previous_i]

            # If t is in the parabola range
            if t <= blend_times[j][d]:
                a = accelerations[j][d]

                qdd[d, i] = a
                qd[d, i] = qd[d, previous_i] + a * t

                if i == 0:
                    q[d, i] = q_[0][d] + 0.5 * a * t * t
                else:
                    q[d, i] = q[d, previous_i] + qd[d, previous_i] * t + 0.5 * a * t * t

                previous_ii = i

            # If t is in the linear range
            elif t <= blend_times[j][d] + time_segments[j][d]:
                t = timesteps[i] - timesteps[previous_ii]
                v = velocities[j][d]

                qdd[d, i] = 0
                qd[d, i] = v
                q[d, i] = q[d, previous_ii] + v * t

            # If t has crossed the parabola plus the linear range
            else:
                previous_i = i - 1
                j += 1

                t = timesteps[i] - timesteps[previous_i]

                # Break loop if parameter exceeded
                if j == len(accelerations):
                    break

                a = accelerations[j][d]
                v = qd[d, previous_i]

                qdd[d, i] = a
                qd[d, i] = v + a * t
                q[d, i] = q[d, previous_i] + v * t + 0.5 * a * t * t

                previous_ii = i

        # Loop over remaining timesteps
        while i < len(timesteps):
            a = accelerations[j - 1][d]
            v = velocities[j - 1][d]

            qdd[d, i] = a
            qd[d, i] = v + a * t
            q[d, i] = q[d, previous_i] + v * t + 0.5 * a * t * t

            i += 1

    return q, qd, qdd