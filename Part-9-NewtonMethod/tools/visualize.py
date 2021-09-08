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

    [0, 0, 0, 1, 1, 1]

    q = ax.quiver(x, y, z, u, v, w, length=0.05, colors = colors, lw=1)

    for i in range(x.shape[0] - 3):
        plt.plot([x[i], x[i+3]], [y[i], y[i+3]], [z[i], z[i+3]], '--', color = 'black')

    plt.show()