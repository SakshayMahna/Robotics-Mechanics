"""
Visualize the transformations

Matplotlib:
quiver plot
"""

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

# Function to plot a single transformation
def plot_transformation(transformation):
    """
    Plot Transformation matrix

    ...

    Parameters
    ---
    transformation: 4x4 transformation matrix

    Returns
    ---
    None

    Notes
    ---
    RGB -> XYZ
    """
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # x, y, z of 6 arrows in a quiver plot
    x = np.array([0, 0, 0, transformation[0, 3], transformation[0, 3], transformation[0, 3]])
    y = np.array([0, 0, 0, transformation[1, 3], transformation[1, 3], transformation[1, 3]])
    z = np.array([0, 0, 0, transformation[2, 3], transformation[2, 3], transformation[2, 3]])

    # u, v, w of 6 arrows in a quiver plot
    u = np.concatenate([np.array([1, 0, 0]), transformation[:3, 0]])
    v = np.concatenate([np.array([0, 1, 0]), transformation[:3, 1]])
    w = np.concatenate([np.array([0, 0, 1]), transformation[:3, 2]])

    # Color(RGB) for 6 arrows, original X, Y, Z and then transformed X, Y, Z
    red = np.array([1, 0, 0])
    green = np.array([0, 1, 0])
    blue = np.array([0, 0, 1])
    colors = np.array([red, green, blue, red, green, blue])

    q = ax.quiver(x, y, z, u, v, w, length=0.05, colors = colors, lw=1)

    plt.plot([x[0], x[3]], [y[0], y[3]], [z[0], z[3]], '--', color = 'black')

    plt.show()


# Function to plot a list of transformations
def plot_transformations(transformations):
    """
    Plot Transformation matrix

    ...

    Parameters
    ---
    transformation: list of 4x4 transformation matrix

    Returns
    ---
    None

    Notes
    ---
    RGB -> XYZ
    """
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    x = np.array([])
    y = np.array([])
    z = np.array([])

    u = np.array([])
    v = np.array([])
    w = np.array([])

    red = np.array([1, 0, 0])
    green = np.array([0, 1, 0])
    blue = np.array([0, 0, 1])
    colors = []

    for transformation in transformations:
        x = np.concatenate([x, [transformation[0, 3], transformation[0, 3], transformation[0, 3]]])
        y = np.concatenate([y, [transformation[1, 3], transformation[1, 3], transformation[1, 3]]])
        z = np.concatenate([z, [transformation[2, 3], transformation[2, 3], transformation[2, 3]]])

        u = np.concatenate([u, transformation[:3, 0]])
        v = np.concatenate([v, transformation[:3, 1]])
        w = np.concatenate([w, transformation[:3, 2]])

        colors.append(red)
        colors.append(green)
        colors.append(blue)

    q = ax.quiver(x, y, z, u, v, w, length=0.05, colors = colors, lw=1)

    for i in range(x.shape[0] - 3):
        plt.plot([x[i], x[i+3]], [y[i], y[i+3]], [z[i], z[i+3]], '--', color = 'black')

    plt.show()

def plot_joint_trajectory(q, qd, qdd):
    """
    Function to plot joint trajectories

    ...

    Parameters
    ---
    q   : Joint Position (Dof x m)
    qd  : Joint Velocity (Dof x m)
    qdd : Joint Acceleration (Dof x m)

    Returns
    ---
    None
    """
    m = q.shape[1]
    timesteps = np.linspace(0, 1, num = m)

    n = q.shape[0]

    fig, axis = plt.subplots(3)
    fig.suptitle("Joint Trajectories")

    # Joint Position Plot
    axis[0].set_title("Position")
    axis[0].set(xlabel = "Time", ylabel = "Position")
    for i in range(n):
        axis[0].plot(timesteps, q[i])

    # Joint Velocity Plot
    axis[1].set_title("Velocity")
    axis[1].set(xlabel = "Time", ylabel = "Velocity")
    for i in range(n):
        axis[1].plot(timesteps, qd[i])

    # Joint Acceleration Plot
    axis[2].set_title("Acceleration")
    axis[2].set(xlabel = "Time", ylabel = "Acceleration")
    for i in range(n):
        axis[2].plot(timesteps, qdd[i])

    # Legends
    legends = [f"Joint_{i + 1}" for i in range(n)]
    axis[0].legend(legends)
    axis[1].legend(legends)
    axis[2].legend(legends)

    fig.tight_layout()
    plt.show()

def plot_points(transformations):
    """
    Function to plot points from transformations

    ...

    Parameters
    ---
    transformations: list of 4x4 transformation matrices

    Returns
    ---
    None
    """
    fig = plt.figure()
    ax = fig.gca(projection = '3d')

    x = []
    y = []
    z = []

    for transformation in transformations:
        x.append(transformation[0, 3])
        y.append(transformation[1, 3])
        z.append(transformation[2, 3])

    ax.scatter3D(x, y, z)

    plt.show()